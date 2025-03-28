package main

import (
	"context"
	"log/slog"
	"net/http"

	"github.com/googlecloudrobotics/ilog"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
)

var (
	// gvr defines which resource we expect to be able to list in the
	// remote cluster. We check for robottypes as the cr-syncer may only be
	// authorized to list syncable & unfiltered resources.
	gvr = schema.GroupVersionResource{
		Group:    "registry.cloudrobotics.com",
		Version:  "v1alpha1",
		Resource: "robottypes",
	}
)

// handler handles HTTP health requests.
type handler struct {
	ctx    context.Context
	client dynamic.Interface
}

func newHealthHandler(ctx context.Context, client dynamic.Interface) http.Handler {
	return &handler{ctx, client}
}

func (h *handler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	// A simple health check: see if we can execute a list request against
	// the apiserver. This might block for a while or fail due to transient
	// network issues, so the liveness probe will need to be tolerant of
	// slow or flaky responses.
	//
	// If this becomes a problem, we could do the requests in the
	// background and just check the status of the latest request here.
	if _, err := h.client.Resource(gvr).List(h.ctx, metav1.ListOptions{Limit: 1}); k8serrors.IsUnauthorized(err) {
		slog.Error("failed health check", ilog.Err(err))
		http.Error(w, "unhealthy", http.StatusInternalServerError)
		return
	}
}
