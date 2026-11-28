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

package main

import (
	"testing"

	"go.opentelemetry.io/otel"
)

func TestSetupTracing_Disabled(t *testing.T) {
	shutdown, err := setupTracing(t.Context(), "", "", 1e-4)
	if err != nil {
		t.Fatalf("setupTracing() returned unexpected error: %v", err)
	}
	if shutdown != nil {
		t.Errorf("setupTracing() returned non-nil shutdown func when tracing disabled")
	}
}

func TestSetupTracing_MutuallyExclusive(t *testing.T) {
	if _, err := setupTracing(t.Context(), "my-project", "oc-agent:4317", 1e-4); err == nil {
		t.Fatal("setupTracing() with both flags succeeded, want error")
	}
}

func TestSetupTracing_OTLP(t *testing.T) {
	prev := otel.GetTracerProvider()
	t.Cleanup(func() { otel.SetTracerProvider(prev) })

	shutdown, err := setupTracing(t.Context(), "", "127.0.0.1:4317", 0.0)
	if err != nil {
		t.Fatalf("setupTracing() failed: %v", err)
	}
	if shutdown == nil {
		t.Fatal("setupTracing() returned nil shutdown func")
	}
	if err := shutdown(t.Context()); err != nil {
		t.Errorf("shutdown() failed: %v", err)
	}
}
