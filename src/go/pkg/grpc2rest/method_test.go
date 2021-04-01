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

package grpc2rest

import (
	"fmt"
	"io/ioutil"
	"log"
	"net/http"
	"net/http/httptest"
	"os"
	"testing"

	"github.com/golang/protobuf/descriptor"
	"github.com/golang/protobuf/proto"
	. "github.com/onsi/gomega"
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/util/yaml"
	"k8s.io/client-go/rest"
)

var (
	helloWorldCrd *crdtypes.CustomResourceDefinition
)

func init() {
	dir, _ := os.Getwd()
	filename := "src/proto/hello-world/service_crd.yaml"
	file, err := os.Open(filename)
	if err != nil {
		log.Fatalf("can't open %s in %s: %v", filename, dir, err)
	}
	crd := crdtypes.CustomResourceDefinition{}
	if err := yaml.NewYAMLOrJSONDecoder(file, 1024*1024).Decode(&crd); err != nil {
		log.Fatalf("can't parse %s: %v", filename, err)
	}
	helloWorldCrd = &crd
}

type mockTransport struct {
	handler func(req *http.Request) (*http.Response, error)

	requestUrl   string
	requestBody  string
	responseCode int
	responseBody string
}

func (t *mockTransport) RoundTrip(req *http.Request) (*http.Response, error) {
	t.requestUrl = req.URL.String()
	if req.Body != nil {
		bodyBytes, err := ioutil.ReadAll(req.Body)
		if err != nil {
			log.Fatal(err)
		}
		t.requestBody = string(bodyBytes)
	} else {
		t.requestBody = ""
	}
	resp := httptest.NewRecorder()
	resp.WriteHeader(t.responseCode)
	resp.WriteString(t.responseBody)
	return resp.Result(), nil
}

func TestRequiresNonEmptyProtoDescriptor(t *testing.T) {
	g := NewGomegaWithT(t)
	crd := helloWorldCrd.DeepCopy()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = ""
	g.Expect(buildMethods(crd, nil)).To(BeEmpty())
}

func TestHandlesBrokenBase64(t *testing.T) {
	g := NewGomegaWithT(t)
	crd := helloWorldCrd.DeepCopy()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = "notbase64!"
	_, err := buildMethods(crd, nil)
	g.Expect(err.Error()).To(ContainSubstring("base64"))
}

func TestHandlesInvalidProtoDescriptor(t *testing.T) {
	g := NewGomegaWithT(t)
	crd := helloWorldCrd.DeepCopy()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = "Zm9vCg=="
	_, err := buildMethods(crd, nil)
	g.Expect(err.Error()).To(ContainSubstring("unmarshal"))
}

func TestServiceNotFound(t *testing.T) {
	g := NewGomegaWithT(t)
	crd := helloWorldCrd.DeepCopy()
	crd.Spec.Names.Kind = "SomeOtherKind"
	_, err := buildMethods(crd, nil)
	g.Expect(err.Error()).To(ContainSubstring("no service with name"))
}

func TestHandlesMultipleVersions(t *testing.T) {
	g := NewGomegaWithT(t)
	crd := helloWorldCrd.DeepCopy()
	crd.Spec.Versions = append(crd.Spec.Versions, crd.Spec.Versions[0])
	_, err := buildMethods(crd, nil)
	g.Expect(err.Error()).To(ContainSubstring("multiple versions"))
}

func createMethodsOrDie(scope crdtypes.ResourceScope) (map[string]Method, *mockTransport) {
	crd := helloWorldCrd.DeepCopy()
	crd.Spec.Scope = scope
	transport := &mockTransport{}
	c := rest.Config{Host: "www.server.com", Transport: transport}
	m, err := buildMethods(crd, &c)
	if err != nil {
		log.Fatalf("unexpected error %v", err)
	}
	return m, transport
}

func helloWorld(method string) string {
	return fmt.Sprintf("/cloudrobotics.hello_world.v1alpha1.K8sHelloWorld/%s", method)
}

var messageTypeTests = []struct {
	method   string
	request  string
	response string
	isWatch  bool
}{
	{"Get", "GetHelloWorldRequest", "HelloWorld", false},
	{"List", "ListHelloWorldRequest", "HelloWorldList", false},
	{"Watch", "WatchHelloWorldRequest", "HelloWorldEvent", true},
	{"Create", "CreateHelloWorldRequest", "HelloWorld", false},
	{"Update", "UpdateHelloWorldRequest", "HelloWorld", false},
	{"Delete", "DeleteHelloWorldRequest", "DeleteHelloWorldResponse", false},
}

func TestRequestHasCorrectMessageTypes(t *testing.T) {
	g := NewGomegaWithT(t)
	methods, _ := createMethodsOrDie(crdtypes.NamespaceScoped)
	for _, tt := range messageTypeTests {
		m := methods[helloWorld(tt.method)]
		_, id := descriptor.ForMessage(m.GetInputMessage().(descriptor.Message))
		g.Expect(*id.Name).To(Equal(tt.request))
		_, od := descriptor.ForMessage(m.GetOutputMessage().(descriptor.Message))
		g.Expect(*od.Name).To(Equal(tt.response))
		g.Expect(m.IsWatchCall()).To(Equal(tt.isWatch))
	}
}

var happyCaseTests = []struct {
	method  string
	message string
	url     string
	body    string
	scope   crdtypes.ResourceScope
}{
	// Empty resourceVersion should set no parameter.
	{
		"Get",
		`name: "foo"`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds/foo",
		"",
		crdtypes.NamespaceScoped,
	},
	// resourceVersion=0 should be propagated because it means "from cache".
	{
		"Get",
		`name: "foo" options: <resourceVersion: "0">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds/foo?resourceVersion=0",
		"",
		crdtypes.NamespaceScoped,
	},
	{
		"Get",
		`name: "foo" options: <resourceVersion: "28">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds/foo?resourceVersion=28",
		"",
		crdtypes.NamespaceScoped,
	},
	// Test whether get in a different namespace works
	{
		"Get",
		`name: "foo" namespace: "nsp" options: <resourceVersion: "28">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/nsp/helloworlds/foo?resourceVersion=28",
		"",
		crdtypes.NamespaceScoped,
	},
	// Test whether get works for a cluster-scoped resource
	{
		"Get",
		`name: "foo" options: <resourceVersion: "28">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/helloworlds/foo?resourceVersion=28",
		"",
		crdtypes.ClusterScoped,
	},
	// List with an empty namespace should list all namespaces.
	{
		"List",
		`options: <resourceVersion: "28">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/helloworlds?resourceVersion=28",
		"",
		crdtypes.NamespaceScoped,
	},
	{
		"List",
		`namespace: "default" options: <resourceVersion: "28">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds?resourceVersion=28",
		"",
		crdtypes.NamespaceScoped,
	},
	// Setting watch: true on List should be ignored.
	{
		"List",
		`namespace: "default" options: <resourceVersion: "28" watch: true>`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds?resourceVersion=28",
		"",
		crdtypes.NamespaceScoped,
	},
	{
		"Watch",
		`namespace: "default" options: <resourceVersion: "28">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds?resourceVersion=28&watch=true",
		"",
		crdtypes.NamespaceScoped,
	},
	// Setting watch: true should make no difference.
	{
		"Watch",
		`namespace: "default" options: <resourceVersion: "28" watch: true>`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds?resourceVersion=28&watch=true",
		"",
		crdtypes.NamespaceScoped,
	},
	{
		"Create",
		`object: <metadata: <name: "foo" creationTimestamp: <seconds: 1551710866> > >`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds",
		`{
			"apiVersion": "hello-world.cloudrobotics.com/v1alpha1",
			"kind": "HelloWorld",
			"metadata": {
				"name": "foo",
				"creationTimestamp": "2019-03-04T14:47:46Z"
			}
		 }`,
		crdtypes.NamespaceScoped,
	},
	// Test whether create in a different namespace works
	{
		"Create",
		`object: <metadata: <name: "foo" namespace: "nsp"> >`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/nsp/helloworlds",
		`{
			"apiVersion": "hello-world.cloudrobotics.com/v1alpha1",
			"kind": "HelloWorld",
			"metadata": {
				"name": "foo",
				"namespace": "nsp"
			}
		 }`,
		crdtypes.NamespaceScoped,
	},
	// Test whether creating a cluster-scoped resource works
	{
		"Create",
		`object: <metadata: <name: "foo"> >`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/helloworlds",
		`{
			"apiVersion": "hello-world.cloudrobotics.com/v1alpha1",
			"kind": "HelloWorld",
			"metadata": {
				"name": "foo"
			}
		 }`,
		crdtypes.ClusterScoped,
	},
	// apiVersion/kind on the input should be ignored.
	{
		"Create",
		`object: <apiVersion: "v1" kind: "Pod" metadata: <name: "foo"> >`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds",
		`{
			"apiVersion": "hello-world.cloudrobotics.com/v1alpha1",
			"kind": "HelloWorld",
			"metadata": {
				"name": "foo"
			}
		 }`,
		crdtypes.NamespaceScoped,
	},
	{
		"Update",
		`object: <metadata: <name: "foo"> >`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds/foo",
		`{
			"apiVersion": "hello-world.cloudrobotics.com/v1alpha1",
			"kind": "HelloWorld",
			"metadata": {
				"name": "foo"
			}
		 }`,
		crdtypes.NamespaceScoped,
	},
	{
		"Delete",
		`name: "foo" options: < propagationPolicy: "Foreground" >`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds/foo",
		`{
			"propagationPolicy": "Foreground"
		}`,
		crdtypes.NamespaceScoped,
	},
}

func TestBuildKubernetesRequestHappyCase(t *testing.T) {
	g := NewGomegaWithT(t)
	for _, tt := range happyCaseTests {
		methods, transport := createMethodsOrDie(tt.scope)
		m := methods[helloWorld(tt.method)]
		msg := m.GetInputMessage()
		g.Expect(proto.UnmarshalText(tt.message, msg)).To(BeNil())
		k8sreq, err := m.BuildKubernetesRequest(msg)
		g.Expect(err).To(BeNil())
		transport.responseCode = http.StatusOK

		k8sreq.DoRaw()

		g.Expect(transport.requestUrl).To(Equal(tt.url))
		if tt.body != "" {
			g.Expect(transport.requestBody).To(MatchJSON(tt.body))
		}
	}
}

var emptyNameTests = []struct {
	method  string
	message string
}{
	{"Get", ""},
	{"Update", "object: <spec: <shouldHello: true>>"},
	{"Delete", ""},
}

func TestBuildKubernetesRequestCatchesEmptyName(t *testing.T) {
	g := NewGomegaWithT(t)
	methods, _ := createMethodsOrDie(crdtypes.NamespaceScoped)
	for _, tt := range emptyNameTests {
		m := methods[helloWorld(tt.method)]
		msg := m.GetInputMessage()
		g.Expect(proto.UnmarshalText(tt.message, msg)).To(BeNil())
		_, err := m.BuildKubernetesRequest(msg)
		g.Expect(err.Error()).To(ContainSubstring("empty name is not allowed"))
	}
}

func TestKubernetesRequestHandlesHttpError(t *testing.T) {
	g := NewGomegaWithT(t)
	methods, transport := createMethodsOrDie(crdtypes.NamespaceScoped)
	m := methods[helloWorld("List")]
	k8sreq, err := m.BuildKubernetesRequest(m.GetInputMessage())
	transport.responseCode = http.StatusGone
	transport.responseBody = `{ "apiVersion": "v1", "kind": "Status", "message": "Foo" }`

	body, err := k8sreq.DoRaw()
	g.Expect(err).ToNot(BeNil())
	g.Expect(err.(*errors.StatusError).Status().Code).To(Equal(int32(http.StatusGone)))
	g.Expect(err.(*errors.StatusError).Error()).To(ContainSubstring("did not return more information"))
	g.Expect(body).To(ContainSubstring("Foo"))
}

func TestStreamWorks(t *testing.T) {
	g := NewGomegaWithT(t)
	methods, transport := createMethodsOrDie(crdtypes.NamespaceScoped)
	m := methods[helloWorld("Watch")]
	transport.responseCode = http.StatusOK
	transport.responseBody = "foo"
	req, _ := m.BuildKubernetesRequest(m.GetInputMessage())

	str, err := req.Stream()
	b, _ := ioutil.ReadAll(str)
	str.Close()

	g.Expect(err).To(BeNil())
	g.Expect(string(b)).To(Equal("foo"))
}

func TestStreamHandlesHttpError(t *testing.T) {
	g := NewGomegaWithT(t)
	methods, transport := createMethodsOrDie(crdtypes.NamespaceScoped)
	m := methods[helloWorld("Watch")]
	req, err := m.BuildKubernetesRequest(m.GetInputMessage())
	transport.responseCode = http.StatusGone
	transport.responseBody = "ignored"

	str, err := req.Stream()

	g.Expect(str).To(BeNil())
	g.Expect(err).ToNot(BeNil())
	g.Expect(err.(*errors.StatusError).Status().Code).To(Equal(int32(http.StatusGone)))
	g.Expect(err.(*errors.StatusError).Error()).To(ContainSubstring("did not return more information"))
}
