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
	"log/slog"
	"net/http"

	"go.opentelemetry.io/contrib/detectors/gcp"
	b3prop "go.opentelemetry.io/contrib/propagators/b3"
	ocprop "go.opentelemetry.io/contrib/propagators/opencensus"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/attribute"
	"go.opentelemetry.io/otel/exporters/otlp/otlptrace/otlptracegrpc"
	"go.opentelemetry.io/otel/propagation"
	"go.opentelemetry.io/otel/sdk/resource"
	sdktrace "go.opentelemetry.io/otel/sdk/trace"
	semconv "go.opentelemetry.io/otel/semconv/v1.41.0"
	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/oauth"
)

// GCP Telemetry API writes the spans to the project in gcp.project_id.
const gcpProjectKey = attribute.Key("gcp.project_id")

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

// IgnoreHealthAndMetrics filters out health/readiness check and metrics scraping paths
// so they do not pollute distributed trace backends.
func IgnoreHealthAndMetrics(r *http.Request) bool {
	switch r.URL.Path {
	case "/healthz", "/readyz", "/metrics":
		return false
	default:
		return true
	}
}

// SetupCloudTracing initializes OpenTelemetry tracing exporting over OTLP gRPC
// to the Google Cloud Telemetry API (telemetry.googleapis.com:443).
func SetupCloudTracing(ctx context.Context, projectID, serviceName string, traceProbability float64) (*sdktrace.TracerProvider, error) {
	if projectID == "" {
		return nil, fmt.Errorf("projectID must be specified")
	}
	creds, err := oauth.NewApplicationDefault(ctx, "https://www.googleapis.com/auth/trace.append")
	if err != nil {
		return nil, fmt.Errorf("load application default credentials: %w", err)
	}
	exporter, err := otlptracegrpc.New(ctx,
		otlptracegrpc.WithEndpoint("telemetry.googleapis.com:443"),
		otlptracegrpc.WithDialOption(grpc.WithPerRPCCredentials(creds)),
	)
	if err != nil {
		return nil, fmt.Errorf("create trace exporter: %w", err)
	}

	resAttrs := []attribute.KeyValue{
		gcpProjectKey.String(projectID),
	}
	if serviceName != "" {
		resAttrs = append(resAttrs, semconv.ServiceName(serviceName))
	}
	res, err := resource.New(ctx,
		resource.WithDetectors(gcp.NewDetector()),
		resource.WithTelemetrySDK(),
		resource.WithHost(),
		resource.WithFromEnv(),
		resource.WithAttributes(resAttrs...),
	)
	if err != nil {
		// resource.New returns a partial resource on failure.
		slog.WarnContext(ctx, "Failed to create telemetry resource", slog.String("error", err.Error()))
	}

	tp := sdktrace.NewTracerProvider(
		sdktrace.WithBatcher(exporter),
		sdktrace.WithResource(res),
		sdktrace.WithSampler(sdktrace.ParentBased(sdktrace.TraceIDRatioBased(traceProbability))),
	)
	otel.SetTracerProvider(tp)
	return tp, nil
}
