package main

import (
	"context"
	"log"
	"net/http"

	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
)

var (
	// gvr defines which resource we expect to be able to list in the
	// remote cluster. We check for robots as the cr-syncer may only be
	// authorized to list syncable resources.
	gvr = schema.GroupVersionResource{"registry.cloudrobotics.com", "v1alpha1", "robots"}
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
	if _, err := h.client.Resource(gvr).List(h.ctx, metav1.ListOptions{}); k8serrors.IsUnauthorized(err) {
		log.Printf("failed health check: %v", err)
		http.Error(w, "unhealthy", http.StatusInternalServerError)
		return
	}
}
