// Copyright 2022 The Cloud Robotics Authors
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
	"context"
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"path"
	"strings"
	"syscall"

	"k8s.io/client-go/kubernetes"
	_ "k8s.io/client-go/plugin/pkg/client/auth"
	"k8s.io/client-go/rest"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/api"
	apiv1 "github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/api/v1"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/k8s"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/memory"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/tokensource"
	"github.com/googlecloudrobotics/ilog"
)

type scopeFlags []string

func (i *scopeFlags) String() string {
	return strings.Join(*i, ",")
}

func (i *scopeFlags) Set(value string) error {
	*i = append(*i, value)
	return nil
}

type KeyStoreOpt string

const (
	Kubernetes = "KUBERNETES"
	Memory     = "IN_MEMORY"
)

// Supported public key backends.
var keyStoreOpts = []string{string(Kubernetes), string(Memory)}

var (
	verbose  = flag.Bool("verbose", false, "DEPRECTAED: Use log_level")
	logLevel = flag.Int("log-level", int(slog.LevelInfo), "the log message level required to be logged")
	// Backend options
	keyStore = flag.String(
		"key-store",
		string(Kubernetes),
		"Public key repository implementation to use. Options: "+strings.Join(keyStoreOpts, ","))
	k8sQPS   = flag.Int("k8s-qps", 25, "Limit of QPS to the Kubernetes API server.")
	k8sBurst = flag.Int("k8s-burst", 50, "Burst limit of QPS to the Kubernetes API server.")
	// API options
	bind     = flag.String("bind", "0.0.0.0", "Address to bind to")
	port     = flag.Int("port", 9090, "Port number to listen on")
	basePath = flag.String("base",
		"/apis/core.token-vendor",
		"Base path where the API will be mounted to.")

	// GCP Cloud options
	project = flag.String("project", "", "The cloud project")

	// Kubernetes backend options
	namespace = flag.String("namespace", "default",
		"The namespace where to store the device keys. (Kubernetes)")

	// Authentication / JWT options
	acceptedAudience = flag.String("accepted_audience",
		"", "Endpoint URL of the token vendor. Used for verification of JWTs send by robots.")
	scopes    = scopeFlags{}
	robotName = flag.String("service_account", "robot-service",
		"Name of the service account to generate cloud access tokens for (unless specified per on-prem robot).")
	allowAnyMethod = flag.Bool("allow_any_method", false,
		"If true, token verification endpoints (token.verify and jwt.verify) allow any HTTP method (GET, POST, etc.) for compatibility with Gateway API ext_authz.")
)

func main() {
	flag.Var(&scopes, "scope", "GCP scopes included in the token given out to robots.")
	flag.Parse()

	ll := slog.Level(*logLevel)
	if *verbose {
		ll = slog.LevelDebug
	}
	logHandler := ilog.NewLogHandler(ll, os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	// init components
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var rep repository.PubKeyRepository
	var err error
	switch *keyStore {
	case Kubernetes:
		config, err := rest.InClusterConfig()
		if err != nil {
			slog.Error("Failed to get config", ilog.Err(err))
			os.Exit(1)
		}
		config.QPS = float32(*k8sQPS)
		config.Burst = *k8sBurst
		cs, err := kubernetes.NewForConfig(config)
		if err != nil {
			slog.Error("Failed to make clientset", ilog.Err(err))
			os.Exit(1)
		}
		if rep, err = k8s.NewK8sRepository(ctx, cs, *namespace); err != nil {
			slog.Error("Failed to make k8s repository client", ilog.Err(err))
			os.Exit(1)
		}
	case Memory:
		if rep, err = memory.NewMemoryRepository(ctx); err != nil {
			slog.Error("Failed to make in-memory repository", ilog.Err(err))
			os.Exit(1)
		}
	default:
		slog.Error("unsupported key store option", slog.String("Value", *keyStore))
		os.Exit(1)
	}
	slog.Info("Set up key store", slog.String("KeyStore", *keyStore))

	verifier, err := oauth.NewTokenVerifier(ctx, &http.Client{}, *project)
	if err != nil {
		slog.Error("Failed to make verifier", ilog.Err(err))
		os.Exit(1)
	}
	ts, err := tokensource.NewGCPTokenSource(ctx, nil, scopes)
	if err != nil {
		slog.Error("Failed to make token source", ilog.Err(err))
		os.Exit(1)
	}
	saName := fmt.Sprintf("%s@%s.iam.gserviceaccount.com", *robotName, *project)
	tv, err := app.NewTokenVendor(ctx, rep, verifier, ts, *acceptedAudience, saName)
	if err != nil {
		slog.Error("Failed to make token vendor", ilog.Err(err))
		os.Exit(1)
	}

	// register API endpoints
	if err := api.Register(); err != nil {
		slog.Error("Failed to register root endpoints", ilog.Err(err))
		os.Exit(1)
	}
	if err := apiv1.Register(tv, path.Join(*basePath, "v1"), &apiv1.Options{AllowAnyMethod: *allowAnyMethod}); err != nil {
		slog.Error("Failed to register v1 endpoints", ilog.Err(err))
		os.Exit(1)
	}

	// serve API
	addr := fmt.Sprintf("%s:%d", *bind, *port)
	if err := api.SetupAndServe(addr); err != nil {
		slog.Error("Failed to listen", slog.String("IP", addr), ilog.Err(err))
		os.Exit(1)
	}
}
