// Copyright 2026 The Cloud Robotics Authors
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

package chartassignment

import (
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
)

// Render loads and expands the chart of a ChartAssignment the same way the
// controller does, without applying the resulting resources to a cluster.
func Render(as *apps.ChartAssignment) ([]*unstructured.Unstructured, error) {
	res, _, err := loadAndExpandChart(as)
	return res, err
}
