// Copyright 2019 The Google Cloud Robotics Authors
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

// Runs a gRPC Pub/Sub server.
// Offers a Cloud PubSub style interface and mirrors all incoming messages to all connected clients.
// The service is used by ros-adapter instances on the robot, cloud and optionally the developer
// workstation.

package main

import (
	"flag"
	"log"
	"net"
	"os"

	"cloud.google.com/go/trace"
	"golang.org/x/net/context"
	pb "google.golang.org/genproto/googleapis/pubsub/v1"
	"google.golang.org/grpc"
)

const (
	maxMsgSizeBytes = 100 * 1024 * 1024
	// Stackdriver Trace paid quota is 10 traces per second. The
	// tunnel handles ~400 messages per second, and we aim for <2 traces
	// per second per tunnel.
	traceFraction      = 0.005
	maxTracesPerSecond = 2
)

var (
	traceClient *trace.Client
)

// runServer runs the server bound to the given address, and optionally reports
// the port it binds to over the given channel.
func runServer(addr string, port chan<- int) {
	log.SetFlags(log.Lshortfile | log.LstdFlags)
	flag.Parse()

	lis, err := net.Listen("tcp", addr)
	if err != nil {
		log.Fatal(err)
	}
	if port != nil {
		port <- lis.(*net.TCPListener).Addr().(*net.TCPAddr).Port
	}

	projectID := os.Getenv("GOOGLE_CLOUD_PROJECT")
	traceClient, err = trace.NewClient(context.Background(), projectID)
	if err != nil {
		log.Fatal(err)
	}
	p, err := trace.NewLimitedSampler(traceFraction, maxTracesPerSecond)
	if err != nil {
		log.Fatal(err)
	}
	traceClient.SetSamplingPolicy(p)

	grpcServer := grpc.NewServer(
		grpc.MaxSendMsgSize(maxMsgSizeBytes),
		grpc.MaxRecvMsgSize(maxMsgSizeBytes))
	pubsubServer := newPubsubServer()
	pb.RegisterPublisherServer(grpcServer, pubsubServer)
	pb.RegisterSubscriberServer(grpcServer, pubsubServer)

	log.Println("Starting tunnel gRPC server...")
	log.Fatal("grpcServer.Serve() ended unexpectedly:", grpcServer.Serve(lis))
}

func main() {
	runServer("[::]:50051", nil)
}
