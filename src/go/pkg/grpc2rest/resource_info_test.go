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
	"time"

	"github.com/golang/protobuf/descriptor"
	"github.com/golang/protobuf/proto"
	. "github.com/onsi/gomega"
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
	"k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset/fake"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/util/yaml"
	"k8s.io/client-go/rest"
)

var crdScope = crdtypes.ClusterScoped

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

func ValidCrd() crdtypes.CustomResourceDefinition {
	dir, _ := os.Getwd()
	filename := "../../../proto/hello-world/service_crd.yaml"
	file, err := os.Open(filename)
	if err != nil {
		log.Fatalf("can't open %s in %s: %v", filename, dir, err)
	}
	crd := crdtypes.CustomResourceDefinition{}
	if err := yaml.NewYAMLOrJSONDecoder(file, 1024*1024).Decode(&crd); err != nil {
		log.Fatalf("can't parse %s: %v", filename, err)
	}
	return crd
}

func SetUpEmpty() (*ResourceInfoRepository, crdclientset.Interface, *mockTransport) {
	transport := &mockTransport{}
	c := rest.Config{Host: "www.server.com", Transport: transport}
	r := NewResourceInfoRepository(&c)
	crds := fake.NewSimpleClientset()
	go func() {
		done := make(chan struct{})
		r.Update(done, crds)
	}()
	return r, crds, transport
}

func SetUpValidCrd(scope crdtypes.ResourceScope) (*ResourceInfoRepository, *mockTransport) {
	r, crds, transport := SetUpEmpty()
	crd := ValidCrd()
	crd.Spec.Scope = scope
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	select {
	case <-r.LogChannel():
		return r, transport
	case err := <-r.ErrorChannel():
		log.Fatalf("expected no error; got %v", err)
		return nil, nil
	}
}

func TestEmptyRepositoryIsError(t *testing.T) {
	r, _, _ := SetUpEmpty()
	_, err := r.GetMethod("/my.namespace/Foo")
	if err == nil {
		t.Errorf("received non-error resource info from empty repository")
	}
}

func TestRequiresNonEmptyProtoDescriptor(t *testing.T) {
	r, crds, _ := SetUpEmpty()
	crd := ValidCrd()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = ""
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	select {
	case err := <-r.ErrorChannel():
		t.Errorf("unexpected error: %v", err)
	case <-time.After(100 * time.Millisecond):
		return
	}
}

func TestHandlesBrokenBase64(t *testing.T) {
	g := NewGomegaWithT(t)
	r, crds, _ := SetUpEmpty()
	crd := ValidCrd()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = "notbase64!"
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("base64"))
}

func TestHandlesInvalidProtoDescriptor(t *testing.T) {
	g := NewGomegaWithT(t)
	r, crds, _ := SetUpEmpty()
	crd := ValidCrd()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = "Zm9vCg=="
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("unmarshal"))
}

func TestServiceNotFound(t *testing.T) {
	g := NewGomegaWithT(t)
	r, crds, _ := SetUpEmpty()
	crd := ValidCrd()
	crd.Spec.Names.Kind = "SomeOtherKind"
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("no service with name"))
}

func TestHandlesMultipleVersions(t *testing.T) {
	g := NewGomegaWithT(t)
	r, crds, _ := SetUpEmpty()
	crd := ValidCrd()
	crd.Spec.Versions = append(crd.Spec.Versions, crd.Spec.Versions[0])
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("multiple versions"))
}

func CreateMethodOrDie(name string, scope crdtypes.ResourceScope) (Method, *mockTransport) {
	r, transport := SetUpValidCrd(scope)
	m, err := r.GetMethod(fmt.Sprintf("/cloudrobotics.hello_world.v1alpha1.K8sHelloWorld/%s", name))
	if err != nil {
		log.Fatalf("unexpected error %v", err)
	}
	return m, transport
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
	for _, tt := range messageTypeTests {
		m, _ := CreateMethodOrDie(tt.method, crdtypes.NamespaceScoped)
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
		`object: <metadata: <name: "foo"> >`,
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
		m, transport := CreateMethodOrDie(tt.method, tt.scope)
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
	for _, tt := range emptyNameTests {
		m, _ := CreateMethodOrDie(tt.method, crdtypes.NamespaceScoped)
		msg := m.GetInputMessage()
		g.Expect(proto.UnmarshalText(tt.message, msg)).To(BeNil())
		_, err := m.BuildKubernetesRequest(msg)
		g.Expect(err.Error()).To(ContainSubstring("empty name is not allowed"))
	}
}

func TestKubernetesRequestHandlesHttpError(t *testing.T) {
	g := NewGomegaWithT(t)
	m, transport := CreateMethodOrDie("List", crdtypes.NamespaceScoped)
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
	m, transport := CreateMethodOrDie("Watch", crdtypes.NamespaceScoped)
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
	m, transport := CreateMethodOrDie("Watch", crdtypes.NamespaceScoped)
	req, err := m.BuildKubernetesRequest(m.GetInputMessage())
	transport.responseCode = http.StatusGone
	transport.responseBody = "ignored"

	str, err := req.Stream()

	g.Expect(str).To(BeNil())
	g.Expect(err).ToNot(BeNil())
	g.Expect(err.(*errors.StatusError).Status().Code).To(Equal(int32(http.StatusGone)))
	g.Expect(err.(*errors.StatusError).Error()).To(ContainSubstring("did not return more information"))
}
