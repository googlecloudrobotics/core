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

// Package telemetry provides shared OpenTelemetry propagators and helpers.
package telemetry

import (
	"context"
	"fmt"

	traceexporter "github.com/GoogleCloudPlatform/opentelemetry-operations-go/exporter/trace"
	b3prop "go.opentelemetry.io/contrib/propagators/b3"
	ocprop "go.opentelemetry.io/contrib/propagators/opencensus"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/attribute"
	"go.opentelemetry.io/otel/propagation"
	"go.opentelemetry.io/otel/sdk/resource"
	sdktrace "go.opentelemetry.io/otel/sdk/trace"
	semconv "go.opentelemetry.io/otel/semconv/v1.41.0"
)

var (
	// HTTPPropagator is a composite propagator for HTTP services supporting
	// traceparent (OTel) and B3 (OpenCensus HTTP).
	HTTPPropagator = propagation.NewCompositeTextMapPropagator(
		propagation.TraceContext{},
		b3prop.New(b3prop.WithInjectEncoding(b3prop.B3MultipleHeader)),
	)

	// GRPCPropagator is a composite propagator for gRPC services supporting
	// traceparent (OTel) and grpc-trace-bin (OpenCensus gRPC).
	GRPCPropagator = propagation.NewCompositeTextMapPropagator(
		propagation.TraceContext{},
		ocprop.Binary{},
	)
)

// SetupCloudTracing initializes OpenTelemetry tracing with Google Cloud Trace exporter.
func SetupCloudTracing(ctx context.Context, projectID, serviceName string, sampler sdktrace.Sampler) (*sdktrace.TracerProvider, error) {
	if projectID == "" {
		return nil, fmt.Errorf("projectID must be specified")
	}
	exporter, err := traceexporter.New(traceexporter.WithProjectID(projectID))
	if err != nil {
		return nil, fmt.Errorf("create trace exporter: %w", err)
	}

	var resAttrs []attribute.KeyValue
	if serviceName != "" {
		resAttrs = append(resAttrs, semconv.ServiceName(serviceName))
	}
	res, err := resource.New(ctx,
		resource.WithAttributes(resAttrs...),
		resource.WithFromEnv(),
		resource.WithTelemetrySDK(),
	)
	if err != nil {
		res = resource.Empty()
	}

	tp := sdktrace.NewTracerProvider(
		sdktrace.WithBatcher(exporter),
		sdktrace.WithResource(res),
		sdktrace.WithSampler(sampler),
	)
	otel.SetTracerProvider(tp)
	return tp, nil
}
