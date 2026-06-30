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

package gcr

import (
	"encoding/json"
	"reflect"
	"testing"
)

func TestDockercfgJSON(t *testing.T) {
	expectedJSON := `{
  "https://gcr.io":{"username":"oauth2accesstoken","password":"ya29.yaddayadda","email":"not@val.id","auth":"b2F1dGgyYWNjZXNzdG9rZW46eWEyOS55YWRkYXlhZGRh"},
  "https://asia.gcr.io":{"username":"oauth2accesstoken","password":"ya29.yaddayadda","email":"not@val.id","auth":"b2F1dGgyYWNjZXNzdG9rZW46eWEyOS55YWRkYXlhZGRh"},
  "https://eu.gcr.io":{"username":"oauth2accesstoken","password":"ya29.yaddayadda","email":"not@val.id","auth":"b2F1dGgyYWNjZXNzdG9rZW46eWEyOS55YWRkYXlhZGRh"},
  "https://us.gcr.io":{"username":"oauth2accesstoken","password":"ya29.yaddayadda","email":"not@val.id","auth":"b2F1dGgyYWNjZXNzdG9rZW46eWEyOS55YWRkYXlhZGRh"}
}`

	gotJSON := DockerCfgJSON("ya29.yaddayadda")

	var expected, got map[string]interface{}
	if err := json.Unmarshal([]byte(expectedJSON), &expected); err != nil {
		t.Fatalf("failed to unmarshal expected JSON: %v", err)
	}
	if err := json.Unmarshal(gotJSON, &got); err != nil {
		t.Fatalf("failed to unmarshal got JSON: %v", err)
	}

	if !reflect.DeepEqual(expected, got) {
		t.Errorf("JSONs do not match.\nExpected: %s\nGot: %s", expectedJSON, string(gotJSON))
	}
}
