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

package main

import (
	"io/ioutil"
	"log"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"strings"
	"testing"
	"time"

	"golang.org/x/oauth2"
)

const (
	writeTimeout = 100 * time.Millisecond
)

func bodyOrDie(r *http.Response) string {
	body, err := ioutil.ReadAll(r.Body)
	if err != nil {
		log.Fatal("Failed to read body stream")
	}
	return string(body)
}

func TestTokenHandlerServesToken(t *testing.T) {
	req := httptest.NewRequest("GET", "/computeMetadata/v1/instance/service-accounts/default/token", strings.NewReader("body"))
	req.RemoteAddr = "192.168.0.101:8001"
	respRecorder := httptest.NewRecorder()
	th := TokenHandler{
		AllowedSources: &net.IPNet{net.IPv4(192, 168, 0, 0), net.CIDRMask(24, 32)},
		TokenSource:    oauth2.StaticTokenSource(&oauth2.Token{AccessToken: "mytoken", Expiry: time.Unix(1531319133, 0), TokenType: "Bearer"}),
		Clock:          func() time.Time { return time.Unix(1531319123, 0) },
	}
	th.ServeHTTP(respRecorder, req)

	if want, got := 200, respRecorder.Result().StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}
	if want, got := "{\"access_token\":\"mytoken\",\"expires_in\":10,\"token_type\":\"Bearer\"}", bodyOrDie(respRecorder.Result()); want != got {
		t.Errorf("Wrong response body; want %s; got %s", want, got)
	}
}

func TestTokenHandlerDeniesWrongAddress(t *testing.T) {
	req := httptest.NewRequest("GET", "/computeMetadata/v1/instance/service-accounts/default/token", strings.NewReader("body"))
	req.RemoteAddr = "192.168.1.101:8001"
	respRecorder := httptest.NewRecorder()
	th := TokenHandler{
		AllowedSources: &net.IPNet{net.IPv4(192, 168, 0, 0), net.CIDRMask(24, 32)},
		TokenSource:    oauth2.StaticTokenSource(&oauth2.Token{AccessToken: "mytoken"}),
	}
	th.ServeHTTP(respRecorder, req)

	if want, got := 403, respRecorder.Result().StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}
}

func TestServiceAccountHandlerReturnsMinimalJSON(t *testing.T) {
	req := httptest.NewRequest("GET", "/computeMetadata/v1/instance/service-accounts/default/?recursive=true", strings.NewReader("body"))
	req.RemoteAddr = "192.168.1.101:8001"
	respRecorder := httptest.NewRecorder()
	sh := ServiceAccountHandler{}
	sh.ServeHTTP(respRecorder, req)

	if want, got := 200, respRecorder.Result().StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}
	if want, got := "{\"aliases\":[],\"email\":\"default\",\"scopes\":[]}", bodyOrDie(respRecorder.Result()); want != got {
		t.Errorf("Wrong response body; want %s; got %s", want, got)
	}
}

func TestMetadataHandlerReturnsZone(t *testing.T) {
	req := httptest.NewRequest("GET", "/computeMetadata/v1/instance/zone", strings.NewReader("body"))
	respRecorder := httptest.NewRecorder()
	mh := MetadataHandler{
		ClusterName:   "robot-robotics",
		ProjectId:     "foo",
		ProjectNumber: 512,
		RobotName:     "28",
		Zone:          "edge",
	}
	mh.ServeHTTP(respRecorder, req)

	if want, got := 200, respRecorder.Result().StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}
	if want, got := "projects/512/zones/edge", bodyOrDie(respRecorder.Result()); want != got {
		t.Errorf("Wrong response body; want %s; got %s", want, got)
	}
}

func TestDetectsDeletionOfFile(t *testing.T) {
	tmpfile, err := ioutil.TempFile("", "tmpfile")
	if err != nil {
		t.Fatal(err)
	}
	if err := tmpfile.Close(); err != nil {
		t.Fatal(err)
	}
	changes := detectChangesToFile(tmpfile.Name())
	if err := os.Remove(tmpfile.Name()); err != nil {
		t.Fatal(err)
	}
	select {
	case <-changes:
		break
	case <-time.After(writeTimeout):
		t.Errorf("no change detected after %s", writeTimeout)
	}
}

func TestNoChangeDetectedWhenFileUnchanged(t *testing.T) {
	tmpfile, err := ioutil.TempFile("", "tmpfile")
	if err != nil {
		t.Fatal(err)
	}
	if err := tmpfile.Close(); err != nil {
		t.Fatal(err)
	}
	defer os.Remove(tmpfile.Name())
	changes := detectChangesToFile(tmpfile.Name())
	select {
	case <-changes:
		t.Errorf("unexpected change detected after %s", writeTimeout)
	case <-time.After(writeTimeout):
	}
}
