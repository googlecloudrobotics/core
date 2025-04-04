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

// The CR syncer syncs custom resources between a remote Kubernetes cluster and
// the local Kubernetes cluster. The spec part is copied from upstream to
// downstream, and the status part is copied from downstream to upstream.
//
// The behaviour can be customized by annotations on the CRDs.
//
// Annotation "filter-by-robot-name"
//
//	cr-syncer.cloudrobotics.com/filter-by-robot-name: <bool>
//
// If true, only sync CRs that have a label 'cloudrobotics.com/robot-name:
// <robot-name>' that matches the robot-name arg given on the command line.
//
// Annotation "status-subtree"
//
//	cr-syncer.cloudrobotics.com/status-subtree: <string>
//
// If specified, only sync the given subtree of the Status field. This is useful
// if resources have a shared status.
//
// Annotation "spec-source"
//
//	cr-syncer.cloudrobotics.com/spec-source: <string>
//
// If set to "cloud", the source of truth for object existence and specs
// (upstream) is the remote cluster and for status it's local (downstream).
// If set to "", the CRD is ignored.
//
// NOTE: Previously, this could be set to "robot", but support was removed as it
// was unused and the required auth setup is more complex, and would need
// changes to cr-syncer-auth-webhook to validate CR creation as well.
package main

import (
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"strings"
	"time"

	"contrib.go.opencensus.io/exporter/prometheus"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"github.com/googlecloudrobotics/ilog"
	"github.com/motemen/go-loghttp"
	"go.opencensus.io/plugin/ochttp"
	"go.opencensus.io/stats/view"
	"go.opencensus.io/tag"
	"go.opencensus.io/zpages"
	"golang.org/x/net/context"
	"golang.org/x/oauth2"
	"golang.org/x/oauth2/google"
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1"
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
	crdinformer "k8s.io/apiextensions-apiserver/pkg/client/informers/externalversions"
	"k8s.io/apimachinery/pkg/watch"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/rest"
	"k8s.io/client-go/tools/cache"
	"k8s.io/klog"
)

const (
	// Resync informers every 5 minutes. This will cause all current resources
	// to be sent as updates once again, which will trigger reconciliation on those
	// objects and thus fix any potential drift.
	resyncPeriod = 5 * time.Minute
)

var (
	remoteServer       = flag.String("remote-server", "", "Remote Kubernetes server")
	robotName          = flag.String("robot-name", "", "Robot we are running on, can be used for selective syncing")
	listenAddr         = flag.String("listen-address", ":80", "HTTP listen address")
	conflictErrorLimit = flag.Int("conflict-error-limit", 5, "Number of consecutive conflict errors before informer is restarted")
	timeout            = flag.Int64("timeout", 300, "Timeout for CR watch calls in seconds")
	useRobotJWT        = flag.Bool("use-robot-jwt", false, "Use robot JWT for authn instead of GCP access token - requires recent CRC cloud deployment")
	verbose            = flag.Bool("verbose", false, "DEPRECATED: Use log_level")
	logLevel           = flag.Int("log-level", int(slog.LevelInfo), "the log message level required to be logged")

	sizeDistribution    = view.Distribution(0, 1024, 2048, 4096, 16384, 65536, 262144, 1048576, 4194304, 33554432)
	latencyDistribution = view.Distribution(0, 1, 2, 5, 10, 15, 25, 50, 100, 200, 400, 800, 1500, 3000, 6000)

	tagLocation = mustNewTagKey("location")
)

func init() {
	if err := view.Register(
		&view.View{
			Name:        ochttp.ClientRequestCount.Name(),
			Description: ochttp.ClientRequestCount.Description(),
			Measure:     ochttp.ClientRequestCount,
			TagKeys:     []tag.Key{ochttp.Method, tagLocation},
			Aggregation: view.Count(),
		},
		&view.View{
			Name:        ochttp.ClientRequestBytes.Name(),
			Description: ochttp.ClientRequestBytes.Description(),
			Measure:     ochttp.ClientRequestBytes,
			TagKeys:     []tag.Key{ochttp.Method, ochttp.StatusCode, tagLocation},
			Aggregation: sizeDistribution,
		},
		&view.View{
			Name:        ochttp.ClientResponseBytes.Name(),
			Description: ochttp.ClientResponseBytes.Description(),
			Measure:     ochttp.ClientResponseBytes,
			TagKeys:     []tag.Key{ochttp.Method, ochttp.StatusCode, tagLocation},
			Aggregation: sizeDistribution,
		},
		&view.View{
			Name:        ochttp.ClientLatency.Name(),
			Description: ochttp.ClientLatency.Description(),
			Measure:     ochttp.ClientLatency,
			TagKeys:     []tag.Key{ochttp.Method, ochttp.StatusCode, tagLocation},
			Aggregation: latencyDistribution,
		},
	); err != nil {
		panic(err)
	}
}

// PrefixingRoundtripper is a HTTP roundtripper that adds a specified prefix to
// all HTTP requests. We need to use it instead of setting APIPath because
// autogenerated and dynamic Kubernetes clients overwrite the REST config's
// APIPath.
type PrefixingRoundtripper struct {
	Prefix string
	Base   http.RoundTripper
}

func (pr *PrefixingRoundtripper) RoundTrip(r *http.Request) (*http.Response, error) {
	// Avoid an extra roundtrip for the protocol upgrade
	r.URL.Scheme = "https"
	if !strings.HasPrefix(r.URL.Path, pr.Prefix+"/") {
		r.URL.Path = pr.Prefix + r.URL.Path
	}
	resp, err := pr.Base.RoundTrip(r)
	return resp, err
}

// ctxRoundTripper injects a fixed context into all requests. This is used to
// provide static OpenCensus tags as Kubernetes' client-go provides no context hooks.
type ctxRoundTripper struct {
	base http.RoundTripper
	ctx  context.Context
}

func (r *ctxRoundTripper) RoundTrip(req *http.Request) (*http.Response, error) {
	return r.base.RoundTrip(req.WithContext(r.ctx))
}

// restConfigForRemote assembles the K8s REST config for the remote server.
func restConfigForRemote(ctx context.Context) (*rest.Config, error) {
	var tokenSource oauth2.TokenSource
	var err error
	if *useRobotJWT {
		tokenSource = robotauth.CreateJWTSource()
	} else {
		tokenSource, err = google.DefaultTokenSource(ctx, "https://www.googleapis.com/auth/cloud-platform")
		if err != nil {
			return nil, err
		}
	}
	ctx, err = tag.New(ctx, tag.Insert(tagLocation, "remote"))
	if err != nil {
		return nil, err
	}
	transport := func(base http.RoundTripper) (rt http.RoundTripper) {
		rt = &oauth2.Transport{
			Source: tokenSource,
			Base:   base,
		}
		rt = &PrefixingRoundtripper{
			Prefix: "/apis/core.kubernetes",
			Base:   rt,
		}
		if *verbose {
			rt = &loghttp.Transport{Transport: rt}
		}
		rt = &ochttp.Transport{Base: rt}
		return &ctxRoundTripper{base: rt, ctx: ctx}
	}
	return &rest.Config{
		Host:          *remoteServer,
		APIPath:       "/apis",
		WrapTransport: transport,
		// The original value of timeout is set in the options of lister and watcher in newInformer function. This timeout is not enforced by the client.
		// That's the reason for the timeout in REST config. It is set to timeout + 5 seconds to give some time for a graceful closing of the connection.
		Timeout: time.Second * (time.Duration(*timeout) + 5),
	}, nil
}

type CrdChange struct {
	Type watch.EventType
	CRD  *crdtypes.CustomResourceDefinition
}

func streamCrds(done <-chan struct{}, clientset crdclientset.Interface, crds chan<- CrdChange) error {
	factory := crdinformer.NewSharedInformerFactory(clientset, 0)
	informer := factory.Apiextensions().V1().CustomResourceDefinitions().Informer()

	go informer.Run(done)

	slog.Info("Syncing cache for CRDs")
	ok := cache.WaitForCacheSync(done, informer.HasSynced)
	if !ok {
		return fmt.Errorf("WaitForCacheSync failed")
	}

	informer.AddEventHandler(cache.ResourceEventHandlerFuncs{
		AddFunc: func(obj interface{}) {
			crds <- CrdChange{Type: watch.Added, CRD: obj.(*crdtypes.CustomResourceDefinition)}
		},
		UpdateFunc: func(oldObj, newObj interface{}) {
			crds <- CrdChange{Type: watch.Modified, CRD: newObj.(*crdtypes.CustomResourceDefinition)}
		},
		DeleteFunc: func(obj interface{}) {
			crds <- CrdChange{Type: watch.Deleted, CRD: obj.(*crdtypes.CustomResourceDefinition)}
		},
	})
	return nil
}

func main() {
	klog.InitFlags(nil)
	flag.Parse()
	ctx := context.Background()

	ll := slog.Level(*logLevel)
	if *verbose {
		ll = slog.LevelDebug
	}
	logHandler := ilog.NewLogHandler(ll, os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	localConfig, err := rest.InClusterConfig()
	if err != nil {
		slog.Error("InClusterConfig", ilog.Err(err))
		os.Exit(1)
	}
	localCtx, err := tag.New(ctx, tag.Insert(tagLocation, "local"))
	if err != nil {
		slog.Error("tag.New", ilog.Err(err))
		os.Exit(1)
	}
	localConfig.WrapTransport = func(base http.RoundTripper) http.RoundTripper {
		if *verbose {
			base = &loghttp.Transport{Transport: base}
		}
		base = &ochttp.Transport{Base: base}
		return &ctxRoundTripper{base: base, ctx: localCtx}
	}
	local, err := dynamic.NewForConfig(localConfig)
	if err != nil {
		slog.Error("NewForConfig", ilog.Err(err))
		os.Exit(1)
	}
	remoteConfig, err := restConfigForRemote(ctx)
	if err != nil {
		slog.Error("restConfigForRemote", ilog.Err(err))
		os.Exit(1)
	}
	remote, err := dynamic.NewForConfig(remoteConfig)
	if err != nil {
		slog.Error("NewForConfig", ilog.Err(err))
		os.Exit(1)
	}

	exporter, err := prometheus.NewExporter(prometheus.Options{})
	if err != nil {
		slog.Error("NewExporter", ilog.Err(err))
		os.Exit(1)
	}
	view.RegisterExporter(exporter)
	view.SetReportingPeriod(time.Second)
	zpages.Handle(nil, "/debug")
	http.Handle("/metrics", exporter)
	http.Handle("/health", newHealthHandler(ctx, remote))

	go func() {
		if err := http.ListenAndServe(*listenAddr, nil); err != nil {
			slog.Error("ListenAndServe", ilog.Err(err))
			os.Exit(1)
		}
	}()

	crds := make(chan CrdChange)
	if err := streamCrds(ctx.Done(), crdclientset.NewForConfigOrDie(localConfig), crds); err != nil {
		slog.Error("Unable to stream CRDs from local Kubernetes", ilog.Err(err))
		os.Exit(1)
	}
	syncers := make(map[string]*crSyncer)
	for crd := range crds {
		name := crd.CRD.GetName()

		if cur, ok := syncers[name]; ok {
			if crd.Type == watch.Added {
				slog.Warn("Already had a running sync", slog.String("syncer", name))
			}
			cur.stop()
			delete(syncers, name)
		}
		if crd.Type == watch.Added || crd.Type == watch.Modified {
			// The modify procedure is very heavyweight: We throw away
			// the informer for the CRD (read: all cached data) on every
			// modification and recreate it. If that ever turns out to
			// be a problem, we should use a shared informer cache
			// instead.
			s, err := newCRSyncer(ctx, *crd.CRD, local, remote, *robotName)
			if err != nil {
				if err != errIgnoredCRD {
					slog.Error("skipping custom resource", slog.String("Resource", name), ilog.Err(err))
				}
				continue
			}
			syncers[name] = s
			go s.run()
		}
	}
}

func mustNewTagKey(s string) tag.Key {
	k, err := tag.NewKey(s)
	if err != nil {
		panic(err)
	}
	return k
}
