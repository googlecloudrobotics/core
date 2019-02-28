// Copyright 2019 The Cloud Robotics Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package main

import (
	"bytes"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"log"
	"net"

	"github.com/golang/protobuf/jsonpb"
	"github.com/googlecloudrobotics/core/src/go/pkg/grpc2rest"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/watch"
	_ "k8s.io/client-go/plugin/pkg/client/auth/gcp"
	"k8s.io/client-go/tools/clientcmd"
)

var (
	resourceInfoRepository *grpc2rest.ResourceInfoRepository
	// Global options for unmarshaling JSON to proto messages.
	// Allow unknown fields as a safety measure (in case the Kubernetes API server's version does
	// not match the version used to generate the proto descriptor).
	unmarshaler = &jsonpb.Unmarshaler{AllowUnknownFields: true}
)

func streamHandler(srv interface{}, stream grpc.ServerStream) error {
	fullMethodName, _ := grpc.MethodFromServerStream(stream)

	method, err := resourceInfoRepository.GetMethod(fullMethodName)
	if err != nil {
		return status.Errorf(codes.InvalidArgument, "no message %s: %v", fullMethodName, err)
	}

	return handleStream(stream, method)
}

func handleStream(stream grpc.ServerStream, method grpc2rest.Method) error {
	inMessage := method.GetInputMessage()

	// Receive proto message.
	if err := stream.RecvMsg(inMessage); err != nil {
		if err == io.EOF {
			return status.Errorf(codes.InvalidArgument, "received no message for unary call")
		}
		return err
	}

	req, err := method.BuildKubernetesRequest(inMessage)
	if err != nil {
		return status.Errorf(codes.InvalidArgument, "failed to build REST request: %v", err)
	}

	// Perform Kubernetes request.
	if method.IsWatchCall() {
		err = watchResponse(stream, method, req)
	} else {
		err = unaryResponse(stream, method, req)
	}
	return err
}

func unaryResponse(stream grpc.ServerStream, method grpc2rest.Method, req grpc2rest.Request) error {
	res, err := req.DoRaw()
	if err != nil {
		// Try reading the status from the response body
		statusMsg := metav1.Status{}
		if err := json.Unmarshal(res, &statusMsg); err == nil {
			return k8sStatusToGRPCStatus(statusMsg).Err()
		}
		return k8sErrorToGRPCError(err)
	}

	// Create instance of dynamic message for received data.
	outMessage := method.GetOutputMessage()

	// Unmarshal kubernetes response to proto message.
	if err := unmarshaler.Unmarshal(bytes.NewReader(res), outMessage); err != nil {
		return status.Errorf(codes.Internal, "error unmarshaling response from Kubernetes: %v\n%s", err, string(res))
	}

	// Send proto message.
	if err := stream.SendMsg(outMessage); err != nil {
		return err
	}

	return nil
}

type WatchEvent struct {
	Type   string
	Object json.RawMessage
}

func watchResponse(stream grpc.ServerStream, method grpc2rest.Method, req grpc2rest.Request) error {
	outMessage := method.GetOutputMessage()
	str, err := req.Stream()
	if err != nil {
		// According to client-go code, Stream() returns no response
		// body on error.
		return k8sErrorToGRPCError(err)
	}
	defer str.Close()

	// Process response stream.
	dec := json.NewDecoder(str)
	for {
		msg := json.RawMessage{}
		if err := dec.Decode(&msg); err != nil {
			if err == io.EOF {
				return nil
			}
			return status.Errorf(codes.Internal, "error getting JSON message from Kubernetes: %v", err)
		}
		event := WatchEvent{}
		if err := json.Unmarshal(msg, &event); err != nil {
			return status.Errorf(codes.Internal, "error unmarshaling watch event: %v", err)
		}
		if event.Type == string(watch.Error) {
			statusMsg := metav1.Status{}
			if err := json.Unmarshal(event.Object, &statusMsg); err != nil {
				return status.Errorf(codes.Internal, "error unmarshaling status: %v", err)
			}
			return k8sStatusToGRPCStatus(statusMsg).Err()
		}
		if err := unmarshaler.Unmarshal(bytes.NewReader(msg), outMessage); err != nil {
			return status.Errorf(codes.Internal, "error parsing Kubernetes message: %v", err)
		}
		if err := stream.SendMsg(outMessage); err != nil {
			return err
		}
		outMessage.Reset()
	}
}

func main() {
	var kubeconfig string
	var master string
	var port int

	flag.StringVar(&kubeconfig, "k", "", "absolute path to the kubeconfig file")
	flag.IntVar(&port, "p", 50053, "listening port of gRPC server")
	flag.StringVar(&master, "m", "", "master URL")

	flag.Parse()

	// Build kubeconfig.
	config, err := clientcmd.BuildConfigFromFlags(master, kubeconfig)
	if err != nil {
		log.Fatalf("error building Kubernetes config: %v", err)
	}

	resourceInfoRepository = grpc2rest.NewResourceInfoRepository(config)
	go func() {
		done := make(chan struct{})
		clientset, err := crdclientset.NewForConfig(config)
		if err != nil {
			log.Fatalf("error building CRD clientset: %v", err)
		}
		if err := resourceInfoRepository.Update(done, clientset); err != nil {
			log.Fatalf("error building resource info repository: %v", err)
		}
	}()
	go func() {
		for {
			select {
			case err := <-resourceInfoRepository.ErrorChannel():
				log.Printf("error in CR definition: %v", err)
			case msg := <-resourceInfoRepository.LogChannel():
				log.Print(msg)
			}
		}
	}()

	// Start gRPC server.
	lis, err := net.Listen("tcp", fmt.Sprintf("localhost:%d", port))
	if err != nil {
		log.Fatalf("failed to listen: %v", err)
	}
	grpcServer := grpc.NewServer(grpc.UnknownServiceHandler(streamHandler))
	log.Fatal(grpcServer.Serve(lis))
}
