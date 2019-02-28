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
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
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

	method, err := resourceInfoRepository.BuildMethod(fullMethodName)
	if err != nil {
		return err
	}

	if method.IsStreamingCall() {
		err = streamingCall(stream, method)
	} else {
		err = unaryCall(stream, method)
	}
	if err != nil {
		log.Print(err)
	}
	return err
}

func unaryCall(stream grpc.ServerStream, method grpc2rest.Method) error {

	inMessage := method.GetInputMessage()

	// Receive proto message.
	if err := stream.RecvMsg(inMessage); err != nil {
		return fmt.Errorf("error receiving message: %v", err)
	}

	req, err := method.BuildKubernetesRequest(inMessage)
	if err != nil {
		return err
	}

	// Perform Kubernetes request.
	log.Printf("Performing request to %s", req.URL().String())
	res, err := req.DoRaw()
	if err != nil {
		return fmt.Errorf("Kubernetes request failed: %v. Response body: %s", err, res)
	}

	// Create instance of dynamic message for received data.
	outMessage := method.GetOutputMessage()

	// Unmarshal kubernetes response to proto message.
	if err := unmarshaler.Unmarshal(bytes.NewReader(res), outMessage); err != nil {
		return fmt.Errorf("error unmarshaling response from Kubernetes: %v\n%s", err, string(res))
	}

	// Send proto message.
	if err := stream.SendMsg(outMessage); err != nil {
		return fmt.Errorf("error sending message: %v", err)
	}

	return nil
}

func streamingCall(stream grpc.ServerStream, method grpc2rest.Method) error {
	// Create dynamic proto message instances for i/o.
	inMessage := method.GetInputMessage()
	outMessage := method.GetOutputMessage()

	// Receive proto message.
	if err := stream.RecvMsg(inMessage); err != nil {
		return fmt.Errorf("error receiving message: %v", err)
	}

	req, err := method.BuildKubernetesRequest(inMessage)
	if err != nil {
		return err
	}

	// Send kubernetes request.
	log.Printf("Performing request to %s", req.URL().String())
	str, err := req.Stream()
	if err != nil {
		return fmt.Errorf("kubernetes request failed: %v", err)
	}
	defer str.Close()

	// Process response stream.
	dec := json.NewDecoder(str)
	for {
		if err := unmarshaler.UnmarshalNext(dec, outMessage); err != nil {
			if err == io.EOF {
				return nil
			}
			return fmt.Errorf("error unmarshaling response from kubernetes: %v", err)
		}
		if err = stream.SendMsg(outMessage); err != nil {
			return fmt.Errorf("error sending message: %v", err)
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
