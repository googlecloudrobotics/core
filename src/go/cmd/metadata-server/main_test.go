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
	"os"
	"testing"
	"time"
)

const (
	writeTimeout = 100 * time.Millisecond
)

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
