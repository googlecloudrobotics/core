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
	"net/http"
	"path"
	"strings"

	log "github.com/sirupsen/logrus"
	"k8s.io/client-go/kubernetes"
	_ "k8s.io/client-go/plugin/pkg/client/auth"
	"k8s.io/client-go/rest"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/api"
	apiv1 "github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/api/v1"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/k8s"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/memory"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/tokensource"
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
	verbose = flag.Bool("verbose", false, "Increase log level to DEBUG.")
	// Backend options
	keyStore = flag.String(
		"key-store",
		string(Kubernetes),
		"Public key repository implementation to use. Options: "+strings.Join(keyStoreOpts, ","))
	k8sQPS = flag.Int("k8s-qps", 25, "Limit of QPS to the Kubernetes API server.")
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
		"Name of the service account to generate cloud access tokens for.")
)

func main() {
	flag.Var(&scopes, "scope", "GCP scopes included in the token given out to robots.")
	flag.Parse()
	if *verbose {
		log.SetLevel(log.DebugLevel)
	} else {
		log.SetLevel(log.InfoLevel)
	}
	// init components
	ctx := context.Background()
	var rep app.PubKeyRepository
	var err error
	if *keyStore == Kubernetes {
		config, err := rest.InClusterConfig()
		if err != nil {
			log.Panic(err)
		}
		config.QPS = float32(*k8sQPS)
		cs, err := kubernetes.NewForConfig(config)
		if err != nil {
			log.Panic(err)
		}
		rep, err = k8s.NewK8sRepository(ctx, cs, *namespace)
		if err != nil {
			log.Panic(err)
		}
	} else if *keyStore == Memory {
		rep, err = memory.NewMemoryRepository(ctx)
		if err != nil {
			log.Panic(err)
		}
	} else {
		log.Panicf("unsupported key store option %q", *keyStore)
	}
	log.Infof("Using key store %q", *keyStore)

	verifier, err := oauth.NewTokenVerifier(ctx, &http.Client{}, *project)
	if err != nil {
		log.Panic(err)
	}
	ts, err := tokensource.NewGCPTokenSource(ctx, nil, *project, *robotName, scopes)
	if err != nil {
		log.Panic(err)
	}
	tv, err := app.NewTokenVendor(ctx, rep, verifier, ts, *acceptedAudience)
	if err != nil {
		log.Panic(err)
	}

	// register API endpoints
	if err := api.Register(); err != nil {
		log.Panic(err)
	}
	if err := apiv1.Register(tv, path.Join(*basePath, "v1")); err != nil {
		log.Panic(err)
	}

	// serve API
	addr := fmt.Sprintf("%s:%d", *bind, *port)
	err = api.SetupAndServe(addr)
	if err != nil {
		log.Panicf("failed to listen on %s: %v", addr, err)
	}
}
