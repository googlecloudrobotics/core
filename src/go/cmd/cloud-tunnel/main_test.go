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

// Test the gRPC server created by the Pub/Sub server.
//
// The Pub/Sub functionality is tested in pubsub_test.go. This just checks that
// the server is responsive.

package main

import (
	"context"
	"fmt"
	"testing"

	pb "google.golang.org/genproto/googleapis/pubsub/v1"

	"google.golang.org/grpc"
)

func TestGRPCServer(t *testing.T) {
	// Start the server on a random port, then connect to it on the port it
	// sends back.
	port := make(chan int, 1)
	go runServer("localhost:0", port)
	grpcAddress := fmt.Sprintf("localhost:%d", <-port)

	conn, err := grpc.Dial(grpcAddress, grpc.WithInsecure())
	if err != nil {
		t.Error("Failed to dial:", err)
		return
	}

	pubClient := pb.NewPublisherClient(conn)
	subClient := pb.NewSubscriberClient(conn)

	_, err = pubClient.CreateTopic(context.Background(), &pb.Topic{Name: "foobar"})
	if err != nil {
		t.Error("CreateTopic failed:", err)
		return
	}

	_, err = subClient.CreateSubscription(context.Background(), &pb.Subscription{
		Name:  "foobar",
		Topic: "foobar",
	})
	if err != nil {
		t.Error("CreateSubscription failed:", err)
		return
	}
}
