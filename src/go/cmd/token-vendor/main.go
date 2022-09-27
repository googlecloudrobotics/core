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
	"log"
	"net/http"
	"path"
	"strings"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/api"
	apiv1 "github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/api/v1"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/cloudiot"
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

var (
	// API options
	bind     = flag.String("bind", "0.0.0.0", "Address to bind to")
	port     = flag.Int("port", 9090, "Port number to listen on")
	basePath = flag.String("base",
		"/apis/core.token-vendor",
		"Base path where the API will be mounted to.")

	// GCP Cloud options
	project = flag.String("project", "", "The cloud project")

	// GCP Cloud IoT core options
	registry = flag.String("registry", "", "The cloud registry")
	region   = flag.String("region", "", "The cloud region")

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
	ctx := context.Background()

	// init components
	r := cloudiot.Registry{Project: *project, Region: *region, Registry: *registry}
	iotreg, err := cloudiot.NewCloudIoTRepository(ctx, r, nil)
	if err != nil {
		log.Panic(err)
	}
	verifier, err := oauth.NewTokenVerifier(ctx, &http.Client{}, *project)
	if err != nil {
		log.Panic(err)
	}
	ts, err := tokensource.NewGCPTokenSource(ctx, nil, *project, *robotName, scopes)
	if err != nil {
		log.Panic(err)
	}
	tv, err := app.NewTokenVendor(ctx, iotreg, verifier, ts, *acceptedAudience)
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
