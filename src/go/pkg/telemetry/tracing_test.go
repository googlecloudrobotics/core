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

package telemetry

import (
	"fmt"
	"net/http"
	"testing"

	"go.opentelemetry.io/otel/propagation"
	"go.opentelemetry.io/otel/trace"
)

func TestHTTPPropagator_InjectAndExtract(t *testing.T) {
	traceID, err := trace.TraceIDFromHex("4bf92f3577b34da6a3ce929d0e0e4736")
	if err != nil {
		t.Fatalf("trace.TraceIDFromHex() failed: %v", err)
	}
	spanID, err := trace.SpanIDFromHex("00f067aa0ba902b7")
	if err != nil {
		t.Fatalf("trace.SpanIDFromHex() failed: %v", err)
	}
	sc := trace.NewSpanContext(trace.SpanContextConfig{
		TraceID:    traceID,
		SpanID:     spanID,
		TraceFlags: trace.FlagsSampled,
	})
	ctx := trace.ContextWithRemoteSpanContext(t.Context(), sc)

	header := http.Header{}
	HTTPPropagator.Inject(ctx, propagation.HeaderCarrier(header))

	wantTraceparent := fmt.Sprintf("00-%s-%s-01", traceID, spanID)
	if got := header.Get("traceparent"); got != wantTraceparent {
		t.Errorf("HTTPPropagator.Inject traceparent mismatch: got %q, want %q", got, wantTraceparent)
	}

	if got := header.Get("X-B3-TraceId"); got != traceID.String() {
		t.Errorf("HTTPPropagator.Inject X-B3-TraceId mismatch: got %q, want %q", got, traceID.String())
	}

	extractedCtx := HTTPPropagator.Extract(t.Context(), propagation.HeaderCarrier(header))
	extractedSC := trace.SpanContextFromContext(extractedCtx)

	if extractedSC.TraceID() != traceID {
		t.Errorf("HTTPPropagator.Extract TraceID mismatch: got %v, want %v", extractedSC.TraceID(), traceID)
	}
	if extractedSC.SpanID() != spanID {
		t.Errorf("HTTPPropagator.Extract SpanID mismatch: got %v, want %v", extractedSC.SpanID(), spanID)
	}
}

func TestGRPCPropagator_InjectAndExtract(t *testing.T) {
	traceID, err := trace.TraceIDFromHex("4bf92f3577b34da6a3ce929d0e0e4736")
	if err != nil {
		t.Fatalf("trace.TraceIDFromHex() failed: %v", err)
	}
	spanID, err := trace.SpanIDFromHex("00f067aa0ba902b7")
	if err != nil {
		t.Fatalf("trace.SpanIDFromHex() failed: %v", err)
	}
	sc := trace.NewSpanContext(trace.SpanContextConfig{
		TraceID:    traceID,
		SpanID:     spanID,
		TraceFlags: trace.FlagsSampled,
	})
	ctx := trace.ContextWithRemoteSpanContext(t.Context(), sc)

	header := http.Header{}
	GRPCPropagator.Inject(ctx, propagation.HeaderCarrier(header))

	wantTraceparent := fmt.Sprintf("00-%s-%s-01", traceID, spanID)
	if got := header.Get("traceparent"); got != wantTraceparent {
		t.Errorf("GRPCPropagator.Inject traceparent mismatch: got %q, want %q", got, wantTraceparent)
	}

	if header.Get("grpc-trace-bin") == "" {
		t.Errorf("GRPCPropagator.Inject grpc-trace-bin missing or empty")
	}

	extractedCtx := GRPCPropagator.Extract(t.Context(), propagation.HeaderCarrier(header))
	extractedSC := trace.SpanContextFromContext(extractedCtx)

	if extractedSC.TraceID() != traceID {
		t.Errorf("GRPCPropagator.Extract TraceID mismatch: got %v, want %v", extractedSC.TraceID(), traceID)
	}
	if extractedSC.SpanID() != spanID {
		t.Errorf("GRPCPropagator.Extract SpanID mismatch: got %v, want %v", extractedSC.SpanID(), spanID)
	}
}
