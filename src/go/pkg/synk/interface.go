package synk

import (
	"context"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
)

type Interface interface {
	Init() error
	Delete(ctx context.Context, name string) error
	Apply(ctx context.Context, name string, opts *ApplyOptions, resources ...*unstructured.Unstructured) (*apps.ResourceSet, error)
}
