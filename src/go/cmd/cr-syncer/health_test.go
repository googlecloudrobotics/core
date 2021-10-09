package main

import (
	"context"
	"net/http"
	"net/http/httptest"
	"testing"

	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic/fake"
	k8stest "k8s.io/client-go/testing"
)

func TestHealthy(t *testing.T) {
	client := fake.NewSimpleDynamicClientWithCustomListKinds(runtime.NewScheme(),
		map[schema.GroupVersionResource]string{
			gvr: "RobotList",
		},
	)
	h := newHealthHandler(context.Background(), client)
	ts := httptest.NewServer(h)
	defer ts.Close()

	res, err := http.Get(ts.URL)
	if err != nil {
		t.Fatal(err)
	}
	if res.StatusCode != http.StatusOK {
		t.Errorf("GET / returned status %d, want %d", res.StatusCode, http.StatusOK)
	}
}

func TestHealthyForBadRequest(t *testing.T) {
	client := fake.NewSimpleDynamicClientWithCustomListKinds(runtime.NewScheme(),
		map[schema.GroupVersionResource]string{
			gvr: "RobotList",
		},
	)
	// To avoid unwanted crashes, we should return "healthy" for misc errors.
	client.PrependReactor("*", "*", func(k8stest.Action) (bool, runtime.Object, error) {
		return true, nil, k8serrors.NewBadRequest("")
	})
	h := newHealthHandler(context.Background(), client)
	ts := httptest.NewServer(h)
	defer ts.Close()

	res, err := http.Get(ts.URL + "/health")
	if err != nil {
		t.Fatal(err)
	}
	if res.StatusCode != http.StatusOK {
		t.Errorf("GET / returned status %d, want %d", res.StatusCode, http.StatusOK)
	}
}

func TestUnhealthy(t *testing.T) {
	client := fake.NewSimpleDynamicClientWithCustomListKinds(runtime.NewScheme(),
		map[schema.GroupVersionResource]string{
			gvr: "RobotList",
		},
	)
	// If the token vendor gives us a bad token, we might get Unauthorized errors.
	// https://github.com/googlecloudrobotics/core/issues/59
	client.PrependReactor("*", "*", func(k8stest.Action) (bool, runtime.Object, error) {
		return true, nil, k8serrors.NewUnauthorized("")
	})
	h := newHealthHandler(context.Background(), client)
	ts := httptest.NewServer(h)
	defer ts.Close()

	res, err := http.Get(ts.URL + "/health")
	if err != nil {
		t.Fatal(err)
	}
	if res.StatusCode != http.StatusInternalServerError {
		t.Errorf("GET / returned status %d, want %d", res.StatusCode, http.StatusInternalServerError)
	}
}
