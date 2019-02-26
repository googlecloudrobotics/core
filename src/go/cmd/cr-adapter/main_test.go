package main

import (
	"io/ioutil"
	"strings"
	"testing"

	helloworld "cloud-robotics.googlesource.com/cloud-robotics/src/proto/hello-world"
	"github.com/golang/mock/gomock"
	"github.com/golang/protobuf/proto"
	. "github.com/onsi/gomega"

	// Necessary because one of the mocks needs the metadata package.
	_ "google.golang.org/grpc/metadata"
)

func TestUnaryCall(t *testing.T) {
	g := NewGomegaWithT(t)
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
	stream.EXPECT().
		RecvMsg(gomock.Any()).
		Return(nil)
	method.EXPECT().
		BuildKubernetesRequest(gomock.Any()).
		Return(restRequest, nil)
	method.EXPECT().
		GetOutputMessage().
		Return(response)
	restResponse := `{ "apiVersion": "hello-world.cloudrobotics.com/v1alpha1", "kind": "HelloWorld", "metadata": { "name": "foo" } }`
	restRequest.EXPECT().
		DoRaw().
		Return([]byte(restResponse), nil)
	stream.EXPECT().
		SendMsg(gomock.Any()).
		Return(nil)

	err := unaryCall(stream, method)
	if err != nil {
		t.Errorf("Expected no error; got %v", err)
	}
	g.Expect(*response.Metadata.Name).To(Equal("foo"))
}

func TestStreamingCall(t *testing.T) {
	g := NewGomegaWithT(t)
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
	stream.EXPECT().
		RecvMsg(gomock.Any()).
		Return(nil)
	method.EXPECT().
		BuildKubernetesRequest(gomock.Any()).
		Return(restRequest, nil)
	method.EXPECT().
		GetOutputMessage().
		Return(response)
	restResponse := `{
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
		}`
	restRequest.EXPECT().
		Stream().
		Return(ioutil.NopCloser(strings.NewReader(restResponse)), nil)
	responses := []*helloworld.HelloWorldEvent{}
	stream.EXPECT().
		SendMsg(gomock.Any()).
		Times(2).
		DoAndReturn(func(msg proto.Message) error {
			responses = append(responses, proto.Clone(msg).(*helloworld.HelloWorldEvent))
			return nil
		})

	err := streamingCall(stream, method)
	if err != nil {
		t.Errorf("Expected no error; got %v", err)
	}
	g.Expect(len(responses)).To(Equal(2))
	g.Expect(*responses[0].Object.Metadata.Name).To(Equal("foo"))
	// Ensure there's no cross-bleed between the two messages.
	g.Expect(responses[0].Object.Spec.ShouldHello).To(Equal(true))
	g.Expect(responses[0].Object.Spec.HellosGiven).To(Equal(int32(0)))
	g.Expect(*responses[1].Object.Metadata.Name).To(Equal("bar"))
	g.Expect(responses[1].Object.Spec.ShouldHello).To(Equal(false))
	g.Expect(responses[1].Object.Spec.HellosGiven).To(Equal(int32(10)))
}
