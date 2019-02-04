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

// Implements the Pub/Sub server.
package main

import (
	"log"
	"sync"
	"time"

	"cloud.google.com/go/trace"
	"github.com/golang/protobuf/ptypes/empty"
	pb "google.golang.org/genproto/googleapis/pubsub/v1"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"

	context "golang.org/x/net/context"
)

// subChanSize is the number of messages in a per-subscription buffer. When
// this buffer is full, messages will be dropped until the subscription starts
// accepting messages again.
//
// TODO(rodrigoq): is this the right approach? What value should this be?
// 100 is copied from the notifier.
const subChanSize = 100

// How long before a blocking pull times out. Cloud Pub/Sub uses ~90s. This
// will be ignored if the context specifies a deadline.
const pullTimeout = 90 * time.Second

// How long before the deadline to send the timeout response. This aims to
// ensure our response arrives before the deadline is exceeded.
const deadlineGracePeriod = 1 * time.Second

type message struct {
	proto *pb.PubsubMessage
	span  *trace.Span
}

type messageSubscription struct {
	proto    *pb.Subscription
	messages chan *message
}

func newMessageSubscription(proto *pb.Subscription) messageSubscription {
	return messageSubscription{
		proto:    proto,
		messages: make(chan *message, subChanSize),
	}
}

type pubsubServer struct {
	sync.RWMutex
	subscriptions map[string]messageSubscription
}

func newPubsubServer() *pubsubServer {
	return &pubsubServer{
		subscriptions: make(map[string]messageSubscription),
	}
}

// handlePublishRequest forwards messages to all matching subscriptions.
func (s *pubsubServer) handlePublishRequest(req *pb.PublishRequest) {
	s.RLock()
	defer s.RUnlock()
	for _, sub := range s.subscriptions {
		if sub.proto.Topic != req.Topic {
			continue
		}
		for _, m := range req.Messages {
			// Non-blocking send avoids interference between subscriptions.
			select {
			case sub.messages <- &message{
				proto: m,
				span:  traceClient.NewSpan("/cloud-tunnel/buffer"),
			}:
			default:
				// TODO(rodrigoq): it would be better to drop
				// the oldest messages rather than the newest
				log.Println("Dropped message(s?) due to full buffer on subscription ", sub.proto.Name)
				break
			}
		}
	}
}

// Creates the given topic with the given name.
func (s *pubsubServer) CreateTopic(ctx context.Context, t *pb.Topic) (*pb.Topic, error) {
	return t, nil
}

// Updates an existing topic. Note that certain properties of a
// topic are not modifiable.
func (s *pubsubServer) UpdateTopic(ctx context.Context, in *pb.UpdateTopicRequest) (*pb.Topic, error) {
	return &pb.Topic{}, nil
}

// Adds one or more messages to the topic. Returns `NOT_FOUND` if the topic
// does not exist. The message payload must not be empty; it must contain
//  either a non-empty data field, or at least one attribute.
func (s *pubsubServer) Publish(ctx context.Context, r *pb.PublishRequest) (*pb.PublishResponse, error) {
	span := traceClient.NewSpan("/cloud-tunnel/publish")
	defer span.Finish()
	s.handlePublishRequest(r)
	ids := make([]string, len(r.Messages))
	for i := range ids {
		// TODO(rodrigoq): IDs should be unique within the topic
		ids[i] = "0"
	}
	return &pb.PublishResponse{
		MessageIds: ids,
	}, nil
}

// Gets the configuration of a topic.
func (s *pubsubServer) GetTopic(ctx context.Context, r *pb.GetTopicRequest) (*pb.Topic, error) {
	return &pb.Topic{}, nil
}

// Lists matching topics.
func (s *pubsubServer) ListTopics(ctx context.Context, r *pb.ListTopicsRequest) (*pb.ListTopicsResponse, error) {
	return &pb.ListTopicsResponse{}, nil
}

// Lists the name of the subscriptions for this topic.
func (s *pubsubServer) ListTopicSubscriptions(ctx context.Context, r *pb.ListTopicSubscriptionsRequest) (*pb.ListTopicSubscriptionsResponse, error) {
	return &pb.ListTopicSubscriptionsResponse{}, nil
}

// Lists the names of the snapshots on this topic.
func (s *pubsubServer) ListTopicSnapshots(ctx context.Context, in *pb.ListTopicSnapshotsRequest) (*pb.ListTopicSnapshotsResponse, error) {
	return &pb.ListTopicSnapshotsResponse{}, nil
}

// Deletes the topic with the given name. Returns `NOT_FOUND` if the topic
// does not exist. After a topic is deleted, a new topic may be created with
// the same name; this is an entirely new topic with none of the old
// configuration or subscriptions. Existing subscriptions to this topic are
// not deleted, but their `topic` field is set to `_deleted-topic_`.
func (s *pubsubServer) DeleteTopic(ctx context.Context, r *pb.DeleteTopicRequest) (*empty.Empty, error) {
	return &empty.Empty{}, nil
}

// Creates a subscription to a given topic.
// If the subscription already exists, returns `ALREADY_EXISTS`.
// If the corresponding topic doesn't exist, returns `NOT_FOUND`.
//
// If the name is not provided in the request, the server will assign a random
// name for this subscription on the same project as the topic, conforming
// to the
// [resource name format](https://cloud.google.com/pubsub/docs/overview#names).
// The generated name is populated in the returned Subscription object.
// Note that for REST API requests, you must specify a name in the request.
func (s *pubsubServer) CreateSubscription(ctx context.Context, sub *pb.Subscription) (*pb.Subscription, error) {
	s.Lock()
	s.subscriptions[sub.Name] = newMessageSubscription(sub)
	s.Unlock()
	return sub, nil
}

// Gets the configuration details of a subscription.
func (s *pubsubServer) GetSubscription(ctx context.Context, r *pb.GetSubscriptionRequest) (*pb.Subscription, error) {
	s.RLock()
	defer s.RUnlock()
	if sub, ok := s.subscriptions[r.Subscription]; ok {
		return sub.proto, nil
	} else {
		return nil, grpc.Errorf(codes.NotFound, "subscription not found")
	}
}

// Updates an existing subscription. Note that certain properties of a
// subscription, such as its topic, are not modifiable.
func (s *pubsubServer) UpdateSubscription(ctx context.Context, r *pb.UpdateSubscriptionRequest) (*pb.Subscription, error) {
	return &pb.Subscription{}, nil
}

// Lists matching subscriptions.
func (s *pubsubServer) ListSubscriptions(ctx context.Context, r *pb.ListSubscriptionsRequest) (*pb.ListSubscriptionsResponse, error) {
	return &pb.ListSubscriptionsResponse{}, nil
}

// Deletes an existing subscription. All messages retained in the subscription
// are immediately dropped. Calls to `Pull` after deletion will return
// `NOT_FOUND`. After a subscription is deleted, a new one may be created with
// the same name, but the new one has no association with the old
// subscription or its topic unless the same topic is specified.
func (s *pubsubServer) DeleteSubscription(ctx context.Context, r *pb.DeleteSubscriptionRequest) (*empty.Empty, error) {
	s.Lock()
	delete(s.subscriptions, r.Subscription)
	s.Unlock()
	return &empty.Empty{}, nil
}

// Modifies the ack deadline for a specific message. This method is useful
// to indicate that more time is needed to process a message by the
// subscriber, or to make the message available for redelivery if the
// processing was interrupted. Note that this does not modify the
// subscription-level `ackDeadlineSeconds` used for subsequent messages.
func (s *pubsubServer) ModifyAckDeadline(ctx context.Context, r *pb.ModifyAckDeadlineRequest) (*empty.Empty, error) {
	return &empty.Empty{}, nil
}

// Acknowledges the messages associated with the `ack_ids` in the
// `AcknowledgeRequest`. The Pub/Sub system can remove the relevant messages
// from the subscription.
//
// Acknowledging a message whose ack deadline has expired may succeed,
// but such a message may be redelivered later. Acknowledging a message more
// than once will not result in an error.
func (s *pubsubServer) Acknowledge(ctx context.Context, r *pb.AcknowledgeRequest) (*empty.Empty, error) {
	return &empty.Empty{}, nil
}

// Pulls messages from the server. Returns an empty list if there are no
// messages available in the backlog. The server may return `UNAVAILABLE` if
// there are too many concurrent pull requests pending for the given
// subscription.
func (s *pubsubServer) Pull(ctx context.Context, r *pb.PullRequest) (*pb.PullResponse, error) {
	// TODO(rodrigoq): return multiple messages if possible
	// TODO(rodrigoq): handle case when subscription is not found
	s.RLock()
	sub := s.subscriptions[r.Subscription]
	s.RUnlock()
	timeout := pullTimeout
	if deadline, ok := ctx.Deadline(); ok {
		timeout = time.Until(deadline) - deadlineGracePeriod
	}
	if r.ReturnImmediately || timeout < 0 {
		timeout = 0
	}
	resp := &pb.PullResponse{}
	// Get at least one message with a blocking read.
	select {
	case m := <-sub.messages:
		resp.ReceivedMessages = append(resp.ReceivedMessages, &pb.ReceivedMessage{
			AckId:   "",
			Message: m.proto,
		})
		m.span.Finish()
	case <-time.After(timeout):
	}
	// Get any waiting messages with non-blocking reads.
get_more_messages:
	for len(resp.ReceivedMessages) < int(r.MaxMessages) {
		select {
		case m := <-sub.messages:
			resp.ReceivedMessages = append(resp.ReceivedMessages, &pb.ReceivedMessage{
				AckId:   "",
				Message: m.proto,
			})
			m.span.Finish()
		default:
			break get_more_messages
		}
	}
	return resp, nil
}

// (EXPERIMENTAL) StreamingPull is an experimental feature. This RPC will
// respond with UNIMPLEMENTED errors unless you have been invited to test
// this feature. Contact cloud-pubsub@google.com with any questions.
//
// Establishes a stream with the server, which sends messages down to the
// client. The client streams acknowledgements and ack deadline modifications
// back to the server. The server will close the stream and return the status
// on any error. The server may close the stream with status `OK` to reassign
// server-side resources, in which case, the client should re-establish the
// stream. `UNAVAILABLE` may also be returned in the case of a transient error
// (e.g., a server restart). These should also be retried by the client. Flow
// control can be achieved by configuring the underlying RPC channel.
func (s *pubsubServer) StreamingPull(srv pb.Subscriber_StreamingPullServer) error {
	return nil
}

// Modifies the `PushConfig` for a specified subscription.
//
// This may be used to change a push subscription to a pull one (signified by
// an empty `PushConfig`) or vice versa, or change the endpoint URL and other
// attributes of a push subscription. Messages will accumulate for delivery
// continuously through the call regardless of changes to the `PushConfig`.
func (s *pubsubServer) ModifyPushConfig(ctx context.Context, r *pb.ModifyPushConfigRequest) (*empty.Empty, error) {
	return &empty.Empty{}, nil
}

// Gets the configuration details of a snapshot.
func (s *pubsubServer) GetSnapshot(ctx context.Context, in *pb.GetSnapshotRequest) (*pb.Snapshot, error) {
	return &pb.Snapshot{}, nil
}

// Lists the existing snapshots.
func (s *pubsubServer) ListSnapshots(ctx context.Context, r *pb.ListSnapshotsRequest) (*pb.ListSnapshotsResponse, error) {
	return &pb.ListSnapshotsResponse{}, nil
}

// Creates a snapshot from the requested subscription.
// If the snapshot already exists, returns `ALREADY_EXISTS`.
// If the requested subscription doesn't exist, returns `NOT_FOUND`.
//
// If the name is not provided in the request, the server will assign a random
// name for this snapshot on the same project as the subscription, conforming
// to the
// [resource name format](https://cloud.google.com/pubsub/docs/overview#names).
// The generated name is populated in the returned Snapshot object.
// Note that for REST API requests, you must specify a name in the request.
func (s *pubsubServer) CreateSnapshot(ctx context.Context, r *pb.CreateSnapshotRequest) (*pb.Snapshot, error) {
	return &pb.Snapshot{}, nil
}

// Updates an existing snapshot.
func (s *pubsubServer) UpdateSnapshot(ctx context.Context, in *pb.UpdateSnapshotRequest) (*pb.Snapshot, error) {
	return &pb.Snapshot{}, nil
}

// Removes an existing snapshot. All messages retained in the snapshot
// are immediately dropped. After a snapshot is deleted, a new one may be
// created with the same name, but the new one has no association with the old
// snapshot or its subscription, unless the same subscription is specified.
func (s *pubsubServer) DeleteSnapshot(ctx context.Context, r *pb.DeleteSnapshotRequest) (*empty.Empty, error) {
	return &empty.Empty{}, nil
}

// Seeks an existing subscription to a point in time or to a given snapshot,
// whichever is provided in the request.
func (s *pubsubServer) Seek(ctx context.Context, r *pb.SeekRequest) (*pb.SeekResponse, error) {
	return &pb.SeekResponse{}, nil
}
