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

// Test the Pub/Sub server without using gRPC.

package main

import (
	"bytes"
	"context"
	"testing"

	pb "google.golang.org/genproto/googleapis/pubsub/v1"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
)

const topicName = "test_topic"
const subName = "test_sub"

// arrays can't be const, but this should not be modified by the tests.
var msgData = []byte("hello")

func TestCreateTopic(t *testing.T) {
	s := newPubsubServer()
	topic, err := s.CreateTopic(context.Background(), &pb.Topic{Name: topicName})
	if err != nil {
		t.Error("CreateTopic failed:", err)
		return
	}

	if topic.Name != topicName {
		t.Errorf("CreateTopic returned the wrong name: %v != %v", topic.Name, topicName)
		return
	}
}

func TestPublish(t *testing.T) {
	s := newPubsubServer()
	_, _ = s.CreateTopic(context.Background(), &pb.Topic{Name: topicName})
	r, err := s.Publish(context.Background(), &pb.PublishRequest{
		Topic: topicName,
		Messages: []*pb.PubsubMessage{
			{
				Data: msgData,
			},
		},
	})
	if err != nil {
		t.Error("Publish failed:", err)
		return
	}
	if len(r.MessageIds) != 1 {
		t.Error("Expected one message ID, got", r.MessageIds)
		return
	}
}

func TestCreateSubscription(t *testing.T) {
	s := newPubsubServer()
	_, _ = s.CreateTopic(context.Background(), &pb.Topic{Name: topicName})
	sub, err := s.CreateSubscription(context.Background(), &pb.Subscription{
		Name:  subName,
		Topic: topicName,
	})

	if err != nil {
		t.Error("CreateSubscription failed:", err)
		return
	}

	if sub.Name != subName {
		t.Errorf("CreateSubscription returned the wrong name: %v != %v", sub.Name, subName)
		return
	}
}

func TestSubscriptionExists(t *testing.T) {
	s := newPubsubServer()
	_, err := s.GetSubscription(context.Background(), &pb.GetSubscriptionRequest{Subscription: subName})
	if grpc.Code(err) != codes.NotFound {
		t.Errorf("Wrong behaviour when getting non-existant subscription, expected codes.NotFound, got %v", err)
		return
	}

	_, _ = s.CreateTopic(context.Background(), &pb.Topic{Name: topicName})
	_, _ = s.CreateSubscription(context.Background(), &pb.Subscription{
		Name:  subName,
		Topic: topicName,
	})

	_, err = s.GetSubscription(context.Background(), &pb.GetSubscriptionRequest{
		Subscription: subName,
	})
	if err != nil {
		t.Errorf("Error when getting subscription: %v", err)
		return
	}
}

func TestOneMessage(t *testing.T) {
	s := newPubsubServer()
	_, _ = s.CreateTopic(context.Background(), &pb.Topic{Name: topicName})
	_, _ = s.CreateSubscription(context.Background(), &pb.Subscription{
		Name:  subName,
		Topic: topicName,
	})
	_, _ = s.Publish(context.Background(), &pb.PublishRequest{
		Topic: topicName,
		Messages: []*pb.PubsubMessage{
			{
				Data: msgData,
			},
		},
	})

	r, err := s.Pull(context.Background(), &pb.PullRequest{
		Subscription:      subName,
		ReturnImmediately: false,
	})
	if err != nil {
		t.Error("Pull failed:", err)
		return
	}
	if len(r.ReceivedMessages) != 1 {
		t.Error("Expected one message, got", r.ReceivedMessages)
		return
	}
	if !bytes.Equal(r.ReceivedMessages[0].Message.Data, msgData) {
		t.Errorf("Expected %s, got %s", msgData, r.ReceivedMessages[0].Message.Data)
		return
	}
}

func TestTwoMessages(t *testing.T) {
	s := newPubsubServer()
	_, _ = s.CreateTopic(context.Background(), &pb.Topic{Name: topicName})
	_, _ = s.CreateSubscription(context.Background(), &pb.Subscription{
		Name:  subName,
		Topic: topicName,
	})
	_, _ = s.Publish(context.Background(), &pb.PublishRequest{
		Topic: topicName,
		Messages: []*pb.PubsubMessage{
			{
				Data: msgData,
			},
			{
				Data: msgData,
			},
		},
	})

	r, err := s.Pull(context.Background(), &pb.PullRequest{
		Subscription:      subName,
		MaxMessages:       2,
		ReturnImmediately: false,
	})
	if err != nil {
		t.Error("Pull failed:", err)
		return
	}
	if len(r.ReceivedMessages) != 2 {
		t.Error("Expected two messages, got", r.ReceivedMessages)
		return
	}
}
