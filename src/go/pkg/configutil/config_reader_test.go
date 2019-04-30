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

package configutil

import (
	"reflect"
	"strings"
	"testing"
)

func TestBashUnescape(t *testing.T) {
	tests := []struct {
		s    string
		want string
	}{
		{
			s:    `foo\ \b\a\r\!`,
			want: `foo bar!`,
		},
		{
			s:    `"foo\ \"bar\"\!"`,
			want: `foo\ "bar"!`,
		},
		{
			s:    `'foo\ '\''bar'\''\!'`,
			want: `foo\ 'bar'\!`,
		},
		{
			s:    `"foo\ \b\a\r\!'`,
			want: `"foo bar!'`,
		},
	}

	for _, tc := range tests {
		if got := bashUnescape(tc.s); got != tc.want {
			t.Errorf("bashUnescape(`%s`) = `%s`', want `%s`", tc.s, got, tc.want)
		}
	}
}

func TestGetConfigFromReader(t *testing.T) {
	s := `
FOO="foo"
  FOO_BAR1=foo\ bar
BAR=buzz
BAR=bar
# NOPE1="don't take this"
NOPE2 = "or this"`
	r := strings.NewReader(s)
	want := map[string]string{
		"FOO":      "foo",
		"FOO_BAR1": "foo bar",
		"BAR":      "bar",
	}

	got, err := getConfigFromReader(r)
	if err != nil {
		t.Fatalf("Got error while reading config: %v", err)
	}
	if !reflect.DeepEqual(got, want) {
		t.Errorf("getConfigFromReader(`%s`) =\n%q\n\nwant:\n%q", s, got, want)
	}
}

func TestSetDefaultVars(t *testing.T) {
	tests := []struct {
		v    map[string]string
		want map[string]string
	}{
		{
			v: map[string]string{
				"GCP_PROJECT_ID": "foo",
			},
			want: map[string]string{
				"GCP_PROJECT_ID":                    "foo",
				"CLOUD_ROBOTICS_CONTAINER_REGISTRY": "gcr.io/foo",
			},
		},
		{
			v: map[string]string{
				"GCP_PROJECT_ID":                    "foo",
				"CLOUD_ROBOTICS_CONTAINER_REGISTRY": "gcr.io/bar",
			},
			want: map[string]string{
				"GCP_PROJECT_ID":                    "foo",
				"CLOUD_ROBOTICS_CONTAINER_REGISTRY": "gcr.io/bar",
			},
		},
	}

	for _, tc := range tests {
		setDefaultVars(tc.v)
		if !reflect.DeepEqual(tc.v, tc.want) {
			t.Errorf("got: %v\nwant %v", tc.v, tc.want)
		}
	}
}
