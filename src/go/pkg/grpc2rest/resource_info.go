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
	"log"
	"sync"

	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
	crdinformer "k8s.io/apiextensions-apiserver/pkg/client/informers/externalversions"
	"k8s.io/client-go/rest"
	"k8s.io/client-go/tools/cache"
)

type ResourceInfoRepository struct {
	config      *rest.Config
	mutex       sync.RWMutex
	grpcMethods map[string]Method
	methodsByCR map[string][]string
}

func NewResourceInfoRepository(config *rest.Config) *ResourceInfoRepository {
	return &ResourceInfoRepository{
		config:      config,
		grpcMethods: make(map[string]Method),
		methodsByCR: make(map[string][]string),
	}
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

func (r *ResourceInfoRepository) insertResourceInfo(obj *crdtypes.CustomResourceDefinition) {
	methods, err := buildMethods(obj, r.config)
	if err != nil {
		log.Printf("adding no methods for invalid CRD %s: %v", obj.GetName(), err)
		return
	}

	log.Printf("adding %d methods for CRD %s", len(methods), obj.GetName())
	names := []string{}
	for k, v := range methods {
		r.grpcMethods[k] = v
		names = append(names, k)
	}
	r.methodsByCR[obj.GetName()] = names
}

func (r *ResourceInfoRepository) deleteResourceInfo(obj *crdtypes.CustomResourceDefinition) {
	methods := r.methodsByCR[obj.GetName()]
	log.Printf("deleting %d methods for CRD %s", len(methods), obj.GetName())
	for _, m := range methods {
		delete(r.grpcMethods, m)
	}
	delete(r.methodsByCR, obj.GetName())
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
			r.insertResourceInfo(obj.(*crdtypes.CustomResourceDefinition))
		},
		UpdateFunc: func(oldObj interface{}, newObj interface{}) {
			r.mutex.Lock()
			defer r.mutex.Unlock()
			r.deleteResourceInfo(oldObj.(*crdtypes.CustomResourceDefinition))
			r.insertResourceInfo(newObj.(*crdtypes.CustomResourceDefinition))
		},
		DeleteFunc: func(obj interface{}) {
			r.mutex.Lock()
			defer r.mutex.Unlock()
			r.deleteResourceInfo(obj.(*crdtypes.CustomResourceDefinition))
		},
	})

	return nil
}
