package main

import (
	"log"
	"os"
	"testing"

	. "github.com/onsi/gomega"
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
	"k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset/fake"
	"k8s.io/apimachinery/pkg/util/yaml"
	"k8s.io/client-go/rest"
)

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

func SetUpEmpty() (*ResourceInfoRepository, crdclientset.Interface) {
	r := NewResourceInfoRepository(&rest.Config{Host: "www.google.com"})
	crds := fake.NewSimpleClientset()
	go func() {
		done := make(chan struct{})
		r.Update(done, crds)
	}()
	return r, crds
}

func SetUpValidCrd() *ResourceInfoRepository {
	r, crds := SetUpEmpty()
	crd := ValidCrd()
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	select {
	case <-r.LogChannel():
		return r
	case err := <-r.ErrorChannel():
		log.Fatalf("expected no error; got %v", err)
		return nil
	}
}

func TestEmptyRepositoryIsError(t *testing.T) {
	r, _ := SetUpEmpty()
	_, err := r.Lookup("my.namespace", "Foo")
	if err == nil {
		t.Errorf("received non-error resource info from empty repository")
	}
}

func TestRequiresNonEmptyProtoDescriptor(t *testing.T) {
	g := NewGomegaWithT(t)
	r, crds := SetUpEmpty()
	crd := ValidCrd()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = ""
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("no proto-descriptor"))
}

func TestHandlesBrokenBase64(t *testing.T) {
	g := NewGomegaWithT(t)
	r, crds := SetUpEmpty()
	crd := ValidCrd()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = "notbase64!"
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("base64"))
}

func TestHandlesInvalidProtoDescriptor(t *testing.T) {
	g := NewGomegaWithT(t)
	r, crds := SetUpEmpty()
	crd := ValidCrd()
	crd.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"] = "Zm9vCg=="
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("unmarshal"))
}

func TestServiceNotFound(t *testing.T) {
	g := NewGomegaWithT(t)
	r, crds := SetUpEmpty()
	crd := ValidCrd()
	crd.Spec.Names.Kind = "SomeOtherKind"
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("no service with name"))
}

func TestHandlesMultipleVersions(t *testing.T) {
	g := NewGomegaWithT(t)
	r, crds := SetUpEmpty()
	crd := ValidCrd()
	crd.Spec.Versions = append(crd.Spec.Versions, crd.Spec.Versions[0])
	crds.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crd)
	err := <-r.ErrorChannel()
	g.Expect(err.Error()).To(ContainSubstring("multiple versions"))
}

func TestSetsStringConstants(t *testing.T) {
	g := NewGomegaWithT(t)
	r := SetUpValidCrd()
	ri, err := r.Lookup("cloudrobotics.hello_world.v1alpha1", "HelloWorld")
	if err != nil {
		t.Errorf("expected HelloWorld ResourceInfo; got %v", err)
	}
	g.Expect(ri.APIVersion).To(Equal("hello-world.cloudrobotics.com/v1alpha1"))
	g.Expect(ri.Kind).To(Equal("HelloWorld"))
	g.Expect(ri.KindPlural).To(Equal("helloworlds"))
}

func TestSetsUpClientCorrectly(t *testing.T) {
	g := NewGomegaWithT(t)
	r := SetUpValidCrd()
	ri, err := r.Lookup("cloudrobotics.hello_world.v1alpha1", "HelloWorld")
	if err != nil {
		t.Errorf("expected HelloWorld ResourceInfo; got %v", err)
	}
	url := ri.Client.Get().Resource(ri.KindPlural).Namespace("default").Name("foo").URL()
	g.Expect(url.String()).To(Equal("http://www.google.com/apis/hello-world.cloudrobotics.com/v1alpha1/namespaces/default/helloworlds/foo"))
}
