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
	"context"
	"errors"
	"io"
	"log/slog"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"strings"
	"testing"
	"time"

	"golang.org/x/oauth2"
)

func bodyOrDie(r *http.Response) string {
	body, err := io.ReadAll(r.Body)
	if err != nil {
		slog.Error("Failed to read body stream")
		os.Exit(1)
	}
	return string(body)
}

func TestConstHandler(t *testing.T) {
	req := httptest.NewRequest("GET", "/url", strings.NewReader("body"))
	respRecorder := httptest.NewRecorder()
	ch := ConstHandler{[]byte("response")}
	ch.ServeHTTP(respRecorder, req)

	if want, got := 200, respRecorder.Result().StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}
	if want, got := "response", bodyOrDie(respRecorder.Result()); want != got {
		t.Errorf("Wrong response body; want %s; got %s", want, got)
	}
}

func TestTokenHandlerServesToken(t *testing.T) {
	oldMinTokenExpiry := *minTokenExpiry
	*minTokenExpiry = 1
	t.Cleanup(func() { *minTokenExpiry = oldMinTokenExpiry })
	testTime := time.Unix(1531319123, 0)
	req := httptest.NewRequest("GET", "/computeMetadata/v1/instance/service-accounts/default/token", strings.NewReader("body"))
	req.RemoteAddr = "192.168.0.101:8001"
	respRecorder := httptest.NewRecorder()
	th := TokenHandler{
		AllowedSources: &net.IPNet{net.IPv4(192, 168, 0, 0), net.CIDRMask(24, 32)},
		TokenSource:    oauth2.StaticTokenSource(&oauth2.Token{AccessToken: "mytoken", Expiry: testTime.Add(10 * time.Second), TokenType: "Bearer"}),
		Clock:          func() time.Time { return testTime },
	}
	th.ServeHTTP(respRecorder, req)

	if want, got := 200, respRecorder.Result().StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}
	if want, got := "{\"access_token\":\"mytoken\",\"expires_in\":10,\"token_type\":\"Bearer\"}", bodyOrDie(respRecorder.Result()); want != got {
		t.Errorf("Wrong response body; want %s; got %s", want, got)
	}
}

type fakeRobotAuth struct {
	ts   oauth2.TokenSource
	id   string
	name string
}

func (a *fakeRobotAuth) CreateRobotTokenSource(context.Context) oauth2.TokenSource {
	return a.ts
}

func (a *fakeRobotAuth) projectID() string {
	return a.id
}

func (a *fakeRobotAuth) robotName() string {
	return a.name
}

func TestTokenHandlerServesLastingToken(t *testing.T) {
	oldMinTokenExpiry := *minTokenExpiry
	*minTokenExpiry = 300
	t.Cleanup(func() { *minTokenExpiry = oldMinTokenExpiry })
	testTime := time.Unix(1531319123, 0)
	req := httptest.NewRequest("GET", "/computeMetadata/v1/instance/service-accounts/default/token", strings.NewReader("body"))
	req.RemoteAddr = "192.168.0.101:8001"
	respRecorder := httptest.NewRecorder()
	th := TokenHandler{
		AllowedSources: &net.IPNet{net.IPv4(192, 168, 0, 0), net.CIDRMask(24, 32)},
		TokenSource:    oauth2.StaticTokenSource(&oauth2.Token{AccessToken: "mytoken", Expiry: testTime.Add(10 * time.Second), TokenType: "Bearer"}),
		Clock:          func() time.Time { return testTime },
		robotAuth: &fakeRobotAuth{
			ts: oauth2.StaticTokenSource(&oauth2.Token{AccessToken: "mytoken", Expiry: testTime.Add(1000 * time.Second), TokenType: "Bearer"}),
		},
	}
	th.ServeHTTP(respRecorder, req)

	if want, got := 200, respRecorder.Result().StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}
	if want, got := "{\"access_token\":\"mytoken\",\"expires_in\":1000,\"token_type\":\"Bearer\"}", bodyOrDie(respRecorder.Result()); want != got {
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
		ClusterName:   "28",
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

var errToken = errors.New("failed to get token")

// fakeTokenSource returns `Errors` consecutive errors then returns tokens.
// Calls counts the number of calls so far.
type fakeTokenSource struct {
	Calls  int
	Errors int
}

func (s *fakeTokenSource) Token() (*oauth2.Token, error) {
	s.Calls = s.Calls + 1
	if s.Calls <= s.Errors {
		return nil, errToken
	}
	return &oauth2.Token{}, nil
}

func TestRateLimitTokenSource(t *testing.T) {
	oldTimeNow := timeNow
	t.Cleanup(func() {
		timeNow = oldTimeNow
	})

	// Test that we retry a certain number of errors within a given amount of
	// time, then succeed. Since we use 100ms steps (not real time), maxTime
	// should not be too large or the test may get slow: One hour can be
	// "simulated" in a few ms, a year takes ~15s.
	tests := []struct {
		desc   string
		errors int
		// Acceptable range of overall duration (too lazy to do the maths to
		// work out exactly how long it should wait).
		minTime time.Duration
		maxTime time.Duration
	}{
		{
			desc:    "no errors",
			errors:  0,
			minTime: 0,
			maxTime: 0,
		},
		{
			desc:    "single error retried within 0.5s",
			errors:  1,
			minTime: 100 * time.Millisecond,
			maxTime: 500 * time.Millisecond,
		},
		{
			desc:    "two errors retried within 1s",
			errors:  2,
			minTime: 200 * time.Millisecond,
			maxTime: time.Second,
		},
		{
			desc: "20 errors retried within 1h",

			errors:  20,
			minTime: 15 * time.Minute,
			maxTime: time.Hour,
		},
	}

	startTime := time.Time{}
	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			clock := startTime
			timeNow = func() time.Time { return clock }

			fakeTS := &fakeTokenSource{Errors: tc.errors}
			ts := newRateLimitTokenSource(fakeTS)
			for ; clock.Sub(startTime) <= tc.maxTime; clock = clock.Add(100 * time.Millisecond) {
				_, err := ts.Token()
				if err != nil {
					continue
				}
				break
			}
			duration := clock.Sub(startTime)
			if duration < tc.minTime {
				t.Errorf("Token() succeeded within %s, want at least %s", duration, tc.minTime)
			}
			if duration > tc.maxTime {
				t.Errorf("Token() did not succeed within %s", tc.maxTime)
			}
		})
	}
}
