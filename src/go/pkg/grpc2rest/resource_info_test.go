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
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
	"k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset/fake"
	"k8s.io/apimachinery/pkg/util/yaml"
	"k8s.io/client-go/rest"
)

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

func SetUpValidCrd() (*ResourceInfoRepository, *mockTransport) {
	r, crds, transport := SetUpEmpty()
	crd := ValidCrd()
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
	g := NewGomegaWithT(t)
	r, crds, _ := SetUpEmpty()
	crd := ValidCrd()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = ""
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("no proto-descriptor"))
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

func CreateMethodOrDie(name string) (Method, *mockTransport) {
	r, transport := SetUpValidCrd()
	m, err := r.GetMethod(fmt.Sprintf("/cloudrobotics.hello_world.v1alpha1.K8sHelloWorld/%s", name))
	if err != nil {
		log.Fatalf("unexpected error %v", err)
	}
	return m, transport
}

var messageTypeTests = []struct {
	method      string
	request     string
	response    string
	isStreaming bool
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
		m, _ := CreateMethodOrDie(tt.method)
		_, id := descriptor.ForMessage(m.GetInputMessage().(descriptor.Message))
		g.Expect(*id.Name).To(Equal(tt.request))
		_, od := descriptor.ForMessage(m.GetOutputMessage().(descriptor.Message))
		g.Expect(*od.Name).To(Equal(tt.response))
		g.Expect(m.IsStreamingCall()).To(Equal(tt.isStreaming))
	}
}

var happyCaseTests = []struct {
	method  string
	message string
	url     string
	body    string
}{
	// Empty resourceVersion should set no parameter.
	{
		"Get",
		`name: "foo"`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds/foo",
		"",
	},
	{
		"Get",
		`name: "foo" options: <resourceVersion: "28">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds/foo?resourceVersion=28",
		"",
	},
	{
		"List",
		`options: <resourceVersion: "28">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds?resourceVersion=28",
		"",
	},
	{
		"Watch",
		`options: <resourceVersion: "28">`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds?resourceVersion=28&watch=true",
		"",
	},
	// Setting watch: true should make no difference.
	{
		"Watch",
		`options: <resourceVersion: "28" watch: true>`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds?resourceVersion=28&watch=true",
		"",
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
	},
	{
		"Delete",
		`name: "foo" options: < propagationPolicy: "Foreground" >`,
		"http://www.server.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds/foo",
		`{
			"propagationPolicy": "Foreground"
		}`,
	},
}

func TestBuildKubernetesRequestHappyCase(t *testing.T) {
	g := NewGomegaWithT(t)
	for _, tt := range happyCaseTests {
		m, transport := CreateMethodOrDie(tt.method)
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
		m, _ := CreateMethodOrDie(tt.method)
		msg := m.GetInputMessage()
		g.Expect(proto.UnmarshalText(tt.message, msg)).To(BeNil())
		_, err := m.BuildKubernetesRequest(msg)
		g.Expect(err.Error()).To(ContainSubstring("empty name is not allowed"))
	}
}

func TestStreamWorks(t *testing.T) {
	g := NewGomegaWithT(t)
	m, transport := CreateMethodOrDie("Watch")
	transport.responseCode = http.StatusOK
	transport.responseBody = "foo"
	req, _ := m.BuildKubernetesRequest(m.GetInputMessage())

	str, err := req.Stream()
	b, _ := ioutil.ReadAll(str)
	str.Close()

	g.Expect(err).To(BeNil())
	g.Expect(string(b)).To(Equal("foo"))
}
