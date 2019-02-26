package main

import (
	"encoding/base64"
	"fmt"
	"log"
	"sync"

	"github.com/golang/protobuf/proto"
	"github.com/golang/protobuf/protoc-gen-go/descriptor"
	"github.com/jhump/protoreflect/desc"
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
	crdinformer "k8s.io/apiextensions-apiserver/pkg/client/informers/externalversions"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/runtime/serializer"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	"k8s.io/client-go/tools/cache"
)

type ResourceInfo struct {
	FileDescriptor *desc.FileDescriptor
	APIVersion     string
	Kind           string
	KindPlural     string
	Client         *rest.RESTClient
}

type ResourceInfoRepository struct {
	mutex     sync.RWMutex
	resources map[string]*ResourceInfo
}

func LookupRepository(protoPackage string, messageName string) (*ResourceInfo, error) {
	key := fmt.Sprintf("%s.K8s%s", protoPackage, messageName)
	resourceInfoRepository.mutex.RLock()
	defer resourceInfoRepository.mutex.RUnlock()
	if v, ok := resourceInfoRepository.resources[key]; ok {
		return v, nil
	}
	return nil, fmt.Errorf("didn't find a CR matching %s", key)
}

func getFileDescriptor(obj *crdtypes.CustomResourceDefinition) (*desc.FileDescriptor, error) {
	annotation := obj.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"]
	if annotation == "" {
		return nil, fmt.Errorf("no proto-descriptor annotation on %s", obj.ObjectMeta.Name)
	}
	b, err := base64.StdEncoding.DecodeString(annotation)
	if err != nil {
		return nil, fmt.Errorf("unable to decode base64 in proto-descriptor annotation on %s: %v", err)
	}
	// Unmarshal file descriptor set.
	fds := &descriptor.FileDescriptorSet{}
	if err := proto.Unmarshal(b, fds); err != nil {
		return nil, fmt.Errorf("unable to unmarshal FileDescriptorSet from proto-descriptor annotation on %s: %v", err)
	}

	// Create dynamic file descriptor.
	fd, err := desc.CreateFileDescriptorFromSet(fds)
	if err != nil {
		return nil, err
	}

	return fd, nil
}

func getServiceName(fd *desc.FileDescriptor, kind string) (string, error) {
	// Determine kind and kindPlural.
	serviceName := fmt.Sprintf("K8s%s", kind)
	svcs := fd.GetServices()
	for _, svc := range svcs {
		if svc.GetName() == serviceName {
			return svc.GetFullyQualifiedName(), nil
		}
	}
	return "", fmt.Errorf("no service with name %s found in proto descriptor for %s", serviceName, kind)
}

func deleteResourceInfo(obj *crdtypes.CustomResourceDefinition) {
	fd, err := getFileDescriptor(obj)
	if err != nil {
		return
	}
	name, err := getServiceName(fd, obj.Spec.Names.Kind)
	if err != nil {
		return
	}
	log.Printf("removing descriptor for service %s", name)
	delete(resourceInfoRepository.resources, name)
}

func insertResourceInfo(obj *crdtypes.CustomResourceDefinition, config *rest.Config) {
	fd, err := getFileDescriptor(obj)
	if err != nil {
		return
	}
	name, err := getServiceName(fd, obj.Spec.Names.Kind)
	if err != nil {
		return
	}

	if len(obj.Spec.Versions) != 1 {
		log.Printf("ignoring CRD %s with multiple versions", obj.ObjectMeta.Name)
		return
	}
	version := obj.Spec.Versions[0].Name

	c := *config
	c.APIPath = "/apis"
	c.GroupVersion = &schema.GroupVersion{obj.Spec.Group, version}
	c.NegotiatedSerializer = serializer.DirectCodecFactory{CodecFactory: scheme.Codecs}

	// Build Kubernetes REST client.
	client, err := rest.RESTClientFor(&c)
	if err != nil {
		log.Fatalf("error building Kubernetes client: %v", err)
	}

	log.Printf("adding descriptor for service %s", name)
	resourceInfoRepository.resources[name] = &ResourceInfo{
		FileDescriptor: fd,
		APIVersion:     c.GroupVersion.String(),
		Kind:           obj.Spec.Names.Kind,
		KindPlural:     obj.Spec.Names.Plural,
		Client:         client,
	}
}

func updateResourceInfoRepository(done <-chan struct{}, config *rest.Config) error {
	clientset, err := crdclientset.NewForConfig(config)
	if err != nil {
		return err
	}
	factory := crdinformer.NewSharedInformerFactory(clientset, 0)
	informer := factory.Apiextensions().V1beta1().CustomResourceDefinitions().Informer()

	go informer.Run(done)

	ok := cache.WaitForCacheSync(done, informer.HasSynced)
	if !ok {
		return fmt.Errorf("WaitForCacheSync failed")
	}

	informer.AddEventHandler(cache.ResourceEventHandlerFuncs{
		AddFunc: func(obj interface{}) {
			resourceInfoRepository.mutex.Lock()
			defer resourceInfoRepository.mutex.Unlock()
			insertResourceInfo(obj.(*crdtypes.CustomResourceDefinition), config)
		},
		UpdateFunc: func(oldObj interface{}, newObj interface{}) {
			resourceInfoRepository.mutex.Lock()
			defer resourceInfoRepository.mutex.Unlock()
			deleteResourceInfo(oldObj.(*crdtypes.CustomResourceDefinition))
			insertResourceInfo(newObj.(*crdtypes.CustomResourceDefinition), config)
		},
		DeleteFunc: func(obj interface{}) {
			resourceInfoRepository.mutex.Lock()
			defer resourceInfoRepository.mutex.Unlock()
			deleteResourceInfo(obj.(*crdtypes.CustomResourceDefinition))
		},
	})

	return nil
}
