package grpc2rest

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

type ResourceInfoRepository struct {
	config      *rest.Config
	logs        chan string
	errors      chan error
	mutex       sync.RWMutex
	grpcMethods map[string]Method
	methodsByCR map[string][]string
}

func NewResourceInfoRepository(config *rest.Config) *ResourceInfoRepository {
	return &ResourceInfoRepository{
		config:      config,
		logs:        make(chan string),
		errors:      make(chan error),
		grpcMethods: make(map[string]Method),
		methodsByCR: make(map[string][]string),
	}
}

func (r *ResourceInfoRepository) LogChannel() <-chan string {
	return r.logs
}

func (r *ResourceInfoRepository) ErrorChannel() <-chan error {
	return r.errors
}

func (r *ResourceInfoRepository) GetMethod(fullMethodName string) (Method, error) {
	r.mutex.RLock()
	defer r.mutex.RUnlock()
	m, ok := r.grpcMethods[fullMethodName]
	if !ok {
		return nil, fmt.Errorf("didn't find a CR with method %s", fullMethodName)
	}
	return m, nil
}

func createRequestParams(method string) (*k8sRequestParams, error) {
	var params *k8sRequestParams

	switch method {
	case "Get":
		params = &k8sRequestParams{
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
	return params, nil
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

func getService(fd *desc.FileDescriptor, kind string) (*desc.ServiceDescriptor, error) {
	// Determine kind and kindPlural.
	serviceName := fmt.Sprintf("K8s%s", kind)
	svcs := fd.GetServices()
	for _, svc := range svcs {
		if svc.GetName() == serviceName {
			return svc, nil
		}
	}
	return nil, fmt.Errorf("no service with name %s found in proto descriptor for %s", serviceName, kind)
}

func (r *ResourceInfoRepository) insertResourceInfo(obj *crdtypes.CustomResourceDefinition) error {
	fd, err := getFileDescriptor(obj)
	if err != nil {
		return err
	}
	svc, err := getService(fd, obj.Spec.Names.Kind)
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

	methods := make(map[string]Method)
	for _, method := range svc.GetMethods() {
		requestParams, err := createRequestParams(method.GetName())
		if err != nil {
			return fmt.Errorf("unrecognized method for %s: %v", method.GetName(), err)
		}
		requestParams.inMessage = dynamic.NewMessage(method.GetInputType())
		requestParams.outMessage = dynamic.NewMessage(method.GetOutputType())
		requestParams.apiVersion = c.GroupVersion.String()
		requestParams.kind = obj.Spec.Names.Kind
		requestParams.kindPlural = obj.Spec.Names.Plural
		requestParams.client = client
		grpcPath := fmt.Sprintf("/%s/%s", svc.GetFullyQualifiedName(), method.GetName())
		methods[grpcPath] = requestParams
	}

	r.logs <- fmt.Sprintf("adding %d methods for service %s", len(methods), svc.GetFullyQualifiedName())
	names := []string{}
	for k, v := range methods {
		r.grpcMethods[k] = v
		names = append(names, k)
	}
	r.methodsByCR[obj.GetName()] = names
	return nil
}

func (r *ResourceInfoRepository) deleteResourceInfo(obj *crdtypes.CustomResourceDefinition) error {
	for _, m := range r.methodsByCR[obj.GetName()] {
		delete(r.grpcMethods, m)
	}
	delete(r.methodsByCR, obj.GetName())
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
