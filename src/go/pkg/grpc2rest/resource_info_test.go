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
	"testing"

	. "github.com/onsi/gomega"
	"k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset/fake"
	"k8s.io/client-go/rest"
)

func TestRepositoryUpdate(t *testing.T) {
	g := NewGomegaWithT(t)
	c := rest.Config{Host: "www.server.com"}
	r := NewResourceInfoRepository(&c)
	crds := fake.NewSimpleClientset()
	done := make(chan struct{})
	r.Update(done, crds)

	hasGet := func() bool {
		m, err := r.GetMethod(helloWorld("Get"))
		return err == nil && m != nil
	}

	g.Expect(hasGet()).To(BeFalse())

	crds.ApiextensionsV1().CustomResourceDefinitions().Create(helloWorldCrd)
	g.Eventually(hasGet).Should(BeTrue())

	crds.ApiextensionsV1().CustomResourceDefinitions().Delete(helloWorldCrd.GetName(), nil)
	g.Eventually(hasGet).Should(BeFalse())
}
