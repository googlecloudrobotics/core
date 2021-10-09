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
	"context"
	"io/ioutil"
	"strings"
	"testing"

	"github.com/golang/mock/gomock"
	"github.com/golang/protobuf/proto"

	helloworld "github.com/googlecloudrobotics/core/src/proto/hello-world"
	. "github.com/onsi/gomega"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"k8s.io/apimachinery/pkg/api/errors"

	// Necessary because the mocks needs these packages.
	_ "google.golang.org/grpc/metadata"
	_ "google.golang.org/protobuf/runtime/protoiface"
)

func doGetCall(t *testing.T, restResponse string, restError error) (*helloworld.HelloWorld, error) {
	ctx := context.Background()
	ctrl := gomock.NewController(t)
	defer ctrl.Finish()
	stream := NewMockServerStream(ctrl)
	method := NewMockMethod(ctrl)
	restRequest := NewMockRequest(ctrl)

	// Use the same request messages as the client would use to ensure
	// compatibility.
	request := &helloworld.GetHelloWorldRequest{Name: "foobar"}
	response := &helloworld.HelloWorld{}
	method.EXPECT().
		GetInputMessage().
		Return(request)
	method.EXPECT().
		IsWatchCall().
		Return(false)
	stream.EXPECT().
		RecvMsg(gomock.Any()).
		Return(nil)
	method.EXPECT().
		BuildKubernetesRequest(gomock.Any()).
		Return(restRequest, nil)
	method.EXPECT().
		GetOutputMessage().
		AnyTimes().
		Return(response)
	restRequest.EXPECT().
		DoRaw(ctx).
		Return([]byte(restResponse), restError)
	expectedResponses := 1
	if restError != nil {
		expectedResponses = 0
	}
	stream.EXPECT().
		SendMsg(gomock.Any()).
		Times(expectedResponses).
		Return(nil)

	err := handleStream(ctx, stream, method)
	return response, err
}

func TestHappyUnaryCall(t *testing.T) {
	g := NewGomegaWithT(t)
	response, err := doGetCall(t, `{ "apiVersion": "hello-world.cloudrobotics.com/v1alpha1", "kind": "HelloWorld", "metadata": { "name": "foo" } }`, nil)
	if err != nil {
		t.Errorf("Expected no error; got %v", err)
	}
	g.Expect(*response.Metadata.Name).To(Equal("foo"))
}

func TestUnaryCallHttpError(t *testing.T) {
	g := NewGomegaWithT(t)
	_, err := doGetCall(t, ``, errors.NewBadRequest("reason"))
	if err == nil {
		t.Errorf("Expected an error")
	}
	g.Expect(status.Code(err)).To(Equal(codes.InvalidArgument))
}

func TestUnaryCallKubernetesErrorTrumpsHttpError(t *testing.T) {
	g := NewGomegaWithT(t)
	_, err := doGetCall(t, `{ "apiVersion": "v1", "kind": "Status", "reason": "InternalError", "message": "FooBar" }`, errors.NewBadRequest("reason"))
	if err == nil {
		t.Errorf("Expected an error")
	}
	g.Expect(status.Code(err)).To(Equal(codes.Internal))
}

func doWatchCall(t *testing.T, restResponse string, restError error) ([]*helloworld.HelloWorldEvent, error) {
	ctx := context.Background()
	ctrl := gomock.NewController(t)
	defer ctrl.Finish()
	stream := NewMockServerStream(ctrl)
	method := NewMockMethod(ctrl)
	restRequest := NewMockRequest(ctrl)

	// Use the same request messages as the client would use to ensure
	// compatibility.
	request := &helloworld.WatchHelloWorldRequest{}
	response := &helloworld.HelloWorldEvent{}
	method.EXPECT().
		GetInputMessage().
		Return(request)
	method.EXPECT().
		IsWatchCall().
		Return(true)
	stream.EXPECT().
		RecvMsg(gomock.Any()).
		Return(nil)
	method.EXPECT().
		BuildKubernetesRequest(gomock.Any()).
		Return(restRequest, nil)
	method.EXPECT().
		GetOutputMessage().
		Return(response)
	restRequest.EXPECT().
		Stream(ctx).
		Return(ioutil.NopCloser(strings.NewReader(restResponse)), restError)
	responses := []*helloworld.HelloWorldEvent{}
	stream.EXPECT().
		SendMsg(gomock.Any()).
		AnyTimes().
		DoAndReturn(func(msg proto.Message) error {
			responses = append(responses, proto.Clone(msg).(*helloworld.HelloWorldEvent))
			return nil
		})

	err := handleStream(ctx, stream, method)
	return responses, err
}

func TestEmptyWatchCall(t *testing.T) {
	g := NewGomegaWithT(t)
	responses, err := doWatchCall(t, ``, nil)
	g.Expect(err).To(BeNil())
	g.Expect(len(responses)).To(Equal(0))
}

func TestHappyWatchCall(t *testing.T) {
	g := NewGomegaWithT(t)
	responses, err := doWatchCall(t, `{
			"type": "ADDED",
			"object": {
				"apiVersion": "hello-world.cloudrobotics.com/v1alpha1",
				"kind": "HelloWorld",
				"metadata": {
					"name": "foo"
				},
				"spec": {
					"shouldHello": true
				}
			}
		}
		{
			"type": "DELETED",
			"object": {
				"apiVersion": "hello-world.cloudrobotics.com/v1alpha1",
				"kind": "HelloWorld",
				"metadata": {
					"name": "bar"
				},
				"spec": {
					"hellosGiven": 10
				}
			}
		}
		`, nil)

	g.Expect(err).To(BeNil())
	g.Expect(len(responses)).To(Equal(2))
	g.Expect(*responses[0].Object.Metadata.Name).To(Equal("foo"))
	// Ensure there's no cross-bleed between the two messages.
	g.Expect(responses[0].Object.Spec.ShouldHello).To(Equal(true))
	g.Expect(responses[0].Object.Spec.HellosGiven).To(Equal(int32(0)))
	g.Expect(*responses[1].Object.Metadata.Name).To(Equal("bar"))
	g.Expect(responses[1].Object.Spec.ShouldHello).To(Equal(false))
	g.Expect(responses[1].Object.Spec.HellosGiven).To(Equal(int32(10)))
}

func TestKubernetesErrorWatchCall(t *testing.T) {
	g := NewGomegaWithT(t)
	responses, err := doWatchCall(t, `{
			"type": "ADDED",
			"object": {}
		}
		{
			"type": "ERROR",
			"object": {
				"kind": "Status",
				"apiVersion": "v1",
				"metadata": {},
				"status": "Failure",
				"message": "api server down",
				"reason": "ServiceUnavailable",
				"code": 503
			}
		}`, nil)

	g.Expect(status.Code(err)).To(Equal(codes.Unavailable))
	g.Expect(len(responses)).To(Equal(1))
}

func TestHttpErrorWatchCall(t *testing.T) {
	g := NewGomegaWithT(t)
	responses, err := doWatchCall(t, "", errors.NewBadRequest("reason"))

	g.Expect(status.Code(err)).To(Equal(codes.InvalidArgument))
	g.Expect(len(responses)).To(Equal(0))
}
