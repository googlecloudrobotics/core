package main

import (
	"encoding/base64"
	"errors"
	"fmt"
	"regexp"
	"sync"

	"github.com/golang/protobuf/proto"
	"github.com/golang/protobuf/protoc-gen-go/descriptor"
	"github.com/jhump/protoreflect/desc"
	"github.com/jhump/protoreflect/dynamic"
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
	crdinformer "k8s.io/apiextensions-apiserver/pkg/client/informers/externalversions"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/runtime/serializer"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	"k8s.io/client-go/tools/cache"
)

var (
	methodRE = regexp.MustCompile("^/(.*)\\.K8s([^.]*)/([^/]*)$")
)

type ResourceInfo struct {
	fileDescriptor *desc.FileDescriptor
	APIVersion     string
	Kind           string
	KindPlural     string
	Client         *rest.RESTClient
}

func (ri *ResourceInfo) GetMessage(name string) (*dynamic.Message, error) {
	md := ri.fileDescriptor.FindMessage(name)
	if md == nil {
		return nil, fmt.Errorf("unknown message: %s", name)
	}
	return dynamic.NewMessage(md), nil
}

type ResourceInfoRepository struct {
	config    *rest.Config
	logs      chan string
	errors    chan error
	mutex     sync.RWMutex
	resources map[string]*ResourceInfo
}

func NewResourceInfoRepository(config *rest.Config) *ResourceInfoRepository {
	return &ResourceInfoRepository{
		config:    config,
		logs:      make(chan string),
		errors:    make(chan error),
		resources: make(map[string]*ResourceInfo),
	}
}

func (r *ResourceInfoRepository) LogChannel() <-chan string {
	return r.logs
}

func (r *ResourceInfoRepository) ErrorChannel() <-chan error {
	return r.errors
}

func (r *ResourceInfoRepository) lookup(protoPackage string, messageName string) (*ResourceInfo, error) {
	key := fmt.Sprintf("%s.K8s%s", protoPackage, messageName)
	r.mutex.RLock()
	defer r.mutex.RUnlock()
	if v, ok := r.resources[key]; ok {
		return v, nil
	}
	return nil, fmt.Errorf("didn't find a CR matching %s", key)
}

func (r *ResourceInfoRepository) BuildMethod(fullMethodName string) (Method, error) {
	streamPkg, streamKind, method, err := parseMethodName(fullMethodName)
	if err != nil {
		return nil, fmt.Errorf("unable to parse method name: %v", err)
	}

	resourceInfo, err := r.lookup(streamPkg, streamKind)
	if err != nil {
		return nil, err
	}

	var params *k8sRequestParams

	switch method {
	case "Get":
		params = &k8sRequestParams{
			inMessageName:        fmt.Sprintf("%s.Get%sRequest", streamPkg, streamKind),
			outMessageName:       fmt.Sprintf("%s.%s", streamPkg, streamKind),
			verb:                 "GET",
			optionsAsQueryParams: true,
			setWatchParam:        false,
			nameInPath:           true,
			setKindAndApiGroup:   false,
			isStreaming:          false,
			bodyFieldName:        "",
		}
	case "List":
		params = &k8sRequestParams{
			inMessageName:        fmt.Sprintf("%s.List%sRequest", streamPkg, streamKind),
			outMessageName:       fmt.Sprintf("%s.%sList", streamPkg, streamKind),
			verb:                 "GET",
			optionsAsQueryParams: true,
			setWatchParam:        false,
			nameInPath:           false,
			setKindAndApiGroup:   false,
			isStreaming:          false,
			bodyFieldName:        "",
		}
	case "Watch":
		params = &k8sRequestParams{
			inMessageName:        fmt.Sprintf("%s.Watch%sRequest", streamPkg, streamKind),
			outMessageName:       fmt.Sprintf("%s.%sEvent", streamPkg, streamKind),
			verb:                 "GET",
			optionsAsQueryParams: true,
			setWatchParam:        true,
			nameInPath:           false,
			setKindAndApiGroup:   false,
			isStreaming:          true,
			bodyFieldName:        "",
		}
	case "Create":
		params = &k8sRequestParams{
			inMessageName:        fmt.Sprintf("%s.Create%sRequest", streamPkg, streamKind),
			outMessageName:       fmt.Sprintf("%s.%s", streamPkg, streamKind),
			verb:                 "POST",
			optionsAsQueryParams: true,
			setWatchParam:        false,
			nameInPath:           false,
			setKindAndApiGroup:   true,
			isStreaming:          false,
			bodyFieldName:        "object",
		}
	case "Update":
		params = &k8sRequestParams{
			inMessageName:        fmt.Sprintf("%s.Update%sRequest", streamPkg, streamKind),
			outMessageName:       fmt.Sprintf("%s.%s", streamPkg, streamKind),
			verb:                 "PUT",
			optionsAsQueryParams: true,
			setWatchParam:        false,
			nameInPath:           true,
			setKindAndApiGroup:   true,
			isStreaming:          false,
			bodyFieldName:        "object",
		}
	case "UpdateStatus":
		return nil, errors.New("UpdateStatus is not yet implemented")
	case "Delete":
		params = &k8sRequestParams{
			inMessageName:        fmt.Sprintf("%s.Delete%sRequest", streamPkg, streamKind),
			outMessageName:       fmt.Sprintf("%s.Delete%sResponse", streamPkg, streamKind),
			verb:                 "DELETE",
			optionsAsQueryParams: false,
			setWatchParam:        false,
			nameInPath:           true,
			setKindAndApiGroup:   false,
			isStreaming:          false,
			bodyFieldName:        "options",
		}
	default:
		return nil, fmt.Errorf("unsupported method: %v", method)
	}

	params.resource = resourceInfo
	return params, nil
}

func parseMethodName(methodName string) (streamPkg string, streamKind string, method string, err error) {
	matches := methodRE.FindStringSubmatch(methodName)
	if len(matches) != 4 {
		err = fmt.Errorf("expected 4 matches in %s", methodName)
		return
	}
	streamPkg, streamKind, method = matches[1], matches[2], matches[3]
	return
}

func getFileDescriptor(obj *crdtypes.CustomResourceDefinition) (*desc.FileDescriptor, error) {
	annotation := obj.ObjectMeta.Annotations["crc.cloudrobotics.com/proto-descriptor"]
	if annotation == "" {
		return nil, fmt.Errorf("no proto-descriptor annotation on %s", obj.ObjectMeta.Name)
	}
	b, err := base64.StdEncoding.DecodeString(annotation)
	if err != nil {
		return nil, fmt.Errorf("unable to decode base64 in proto-descriptor annotation on %s: %v", obj.ObjectMeta.Name, err)
	}
	// Unmarshal file descriptor set.
	fds := &descriptor.FileDescriptorSet{}
	if err := proto.Unmarshal(b, fds); err != nil {
		return nil, fmt.Errorf("unable to unmarshal FileDescriptorSet from proto-descriptor annotation on %s: %v", obj.ObjectMeta.Name, err)
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

func (r *ResourceInfoRepository) insertResourceInfo(obj *crdtypes.CustomResourceDefinition) error {
	fd, err := getFileDescriptor(obj)
	if err != nil {
		return err
	}
	name, err := getServiceName(fd, obj.Spec.Names.Kind)
	if err != nil {
		return err
	}

	if len(obj.Spec.Versions) != 1 {
		return fmt.Errorf("ignoring CRD %s with multiple versions", obj.ObjectMeta.Name)
	}
	version := obj.Spec.Versions[0].Name

	c := *r.config
	c.APIPath = "/apis"
	c.GroupVersion = &schema.GroupVersion{obj.Spec.Group, version}
	c.NegotiatedSerializer = serializer.DirectCodecFactory{CodecFactory: scheme.Codecs}

	// Build Kubernetes REST client.
	client, err := rest.RESTClientFor(&c)
	if err != nil {
		return fmt.Errorf("error building Kubernetes client: %v", err)
	}

	r.resources[name] = &ResourceInfo{
		fileDescriptor: fd,
		APIVersion:     c.GroupVersion.String(),
		Kind:           obj.Spec.Names.Kind,
		KindPlural:     obj.Spec.Names.Plural,
		Client:         client,
	}

	r.logs <- fmt.Sprintf("adding descriptor for service %s", name)
	return nil
}

func (r *ResourceInfoRepository) deleteResourceInfo(obj *crdtypes.CustomResourceDefinition) error {
	fd, err := getFileDescriptor(obj)
	if err != nil {
		return err
	}
	name, err := getServiceName(fd, obj.Spec.Names.Kind)
	if err != nil {
		return err
	}
	r.logs <- fmt.Sprintf("removing descriptor for service %s", name)
	delete(r.resources, name)
	return nil
}

func (r *ResourceInfoRepository) Update(done <-chan struct{}, clientset crdclientset.Interface) error {
	factory := crdinformer.NewSharedInformerFactory(clientset, 0)
	informer := factory.Apiextensions().V1beta1().CustomResourceDefinitions().Informer()

	go informer.Run(done)

	ok := cache.WaitForCacheSync(done, informer.HasSynced)
	if !ok {
		return fmt.Errorf("WaitForCacheSync failed")
	}

	informer.AddEventHandler(cache.ResourceEventHandlerFuncs{
		AddFunc: func(obj interface{}) {
			r.mutex.Lock()
			defer r.mutex.Unlock()
			if err := r.insertResourceInfo(obj.(*crdtypes.CustomResourceDefinition)); err != nil {
				r.errors <- err
			}
		},
		UpdateFunc: func(oldObj interface{}, newObj interface{}) {
			r.mutex.Lock()
			defer r.mutex.Unlock()
			if err := r.deleteResourceInfo(oldObj.(*crdtypes.CustomResourceDefinition)); err != nil {
				r.errors <- err
			}
			if err := r.insertResourceInfo(newObj.(*crdtypes.CustomResourceDefinition)); err != nil {
				r.errors <- err
			}
		},
		DeleteFunc: func(obj interface{}) {
			r.mutex.Lock()
			defer r.mutex.Unlock()
			if err := r.deleteResourceInfo(obj.(*crdtypes.CustomResourceDefinition)); err != nil {
				r.errors <- err
			}
		},
	})

	return nil
}
