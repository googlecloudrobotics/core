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

package setup

import (
	"testing"

	"github.com/golang/mock/gomock"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
)

func TestSelectRobot(t *testing.T) {
	mockCtrl := gomock.NewController(t)
	defer mockCtrl.Finish()

	mockFactory := NewMockFactory(mockCtrl)

	robots := []unstructured.Unstructured{
		{
			Object: map[string]interface{}{
				"apiVersion": "registry.cloudrobotics.com/v1alpha1",
				"kind":       "Robot",
				"metadata": map[string]interface{}{
					"namespace": "default",
					"name":      "ro-1234",
					"labels": map[string]interface{}{
						"cloudrobotics.com/robot-name": "ro-1234",
					},
				},
				"spec": map[string]interface{}{
					"type": "test",
					"role": "test",
				},
			},
		},
	}

	mockFactory.EXPECT().ScanInt().Return(1, nil).Times(1)

	id, err := selectRobot(mockFactory, robots)
	if id != "ro-1234" || err != nil {
		t.Errorf("selectRobot(mockFactory, oneRobot) = %v, %v want ro-1234, nil", id, err)
	}
}
