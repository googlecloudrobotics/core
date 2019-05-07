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

	for i, tc := range tests {
		if got := bashUnescape(tc.s); got != tc.want {
			t.Errorf("[%d] bashUnescape(`%s`) = `%s`', want `%s`", i, tc.s, got, tc.want)
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

	for i, tc := range tests {
		setDefaultVars(tc.v)
		if !reflect.DeepEqual(tc.v, tc.want) {
			t.Errorf("[%d], got: %v\nwant %v", i, tc.v, tc.want)
		}
	}
}

func TestGetBoolean(t *testing.T) {
	tests := []struct {
		v    map[string]string
		def  bool
		want bool
	}{
		{ // good key present
			v:    map[string]string{"FLAG": "true"},
			def:  false,
			want: true,
		},
		{ // no values, return def
			v:    map[string]string{},
			def:  false,
			want: false,
		},
		{ // key absent, return def
			v:    map[string]string{"OPTION": "true"},
			def:  false,
			want: false,
		},
		{ // good key, but bad value present, return def
			v:    map[string]string{"FLAG": "I am not a flag"},
			def:  false,
			want: false,
		},
	}

	for i, tc := range tests {
		r := GetBoolean(tc.v, "FLAG", tc.def)
		if r != tc.want {
			t.Errorf("[%d] got: %v\nwant %v", i, r, tc.want)
		}
	}
}
