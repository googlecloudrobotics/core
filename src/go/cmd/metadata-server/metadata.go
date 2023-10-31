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

// Package metadata implements the metadat http handlers.
package main

import (
	"context"
	"encoding/json"
	"fmt"
	"hash/fnv"
	"log"
	"net"
	"net/http"
	"strings"
	"sync"
	"time"

	"github.com/cenkalti/backoff"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"golang.org/x/oauth2"
	cloudresourcemanager "google.golang.org/api/cloudresourcemanager/v1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes"
)

const (
	// getPodByIPRetries configures how often we retry determining the ip for a pod
	getPodByIPRetries = 10
	// getPodByIPWait confgured the time to sleep between the retries
	getPodByIPWait = 500 * time.Millisecond
)

// rateLimitTokenSource is a TokenSource that applies exponential backoff on
// errors, returning the previous error if called again too soon. It doesn't
// rate-limit on success, as it assumes it wraps an oauth2.ReuseTokenSource.
type rateLimitTokenSource struct {
	wrapped oauth2.TokenSource

	mu    sync.Mutex // guards err and next
	err   error
	delay time.Duration
	next  time.Time
}

func newRateLimitTokenSource(ts oauth2.TokenSource) *rateLimitTokenSource {
	rlts := &rateLimitTokenSource{wrapped: ts}
	rlts.resetBackoff()
	return rlts
}

var timeNow = time.Now

// Token tries to fetch a token, unless an error was recently encountered, in
// which case the previous error is returned.
func (s *rateLimitTokenSource) Token() (*oauth2.Token, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if timeNow().Before(s.next) {
		return nil, s.err
	}
	t, err := s.wrapped.Token()
	if err != nil {
		s.updateBackoff(err)
		return nil, err
	}
	s.resetBackoff()
	return t, nil
}

func (s *rateLimitTokenSource) resetBackoff() {
	s.delay = 100 * time.Millisecond
	s.next = time.Time{}
}

func (s *rateLimitTokenSource) updateBackoff(err error) {
	s.next = timeNow().Add(s.delay)
	s.err = err
	if s.delay < 120*time.Second {
		s.delay *= 2
	}
}

// ConstHandler serves OK responses with static body content.
type ConstHandler struct {
	Body []byte
}

func (ch ConstHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Metadata-Flavor", "Google")
	w.Header().Set("Content-Type", "application/text")
	w.WriteHeader(http.StatusOK)
	w.Write(ch.Body)
}

// TokenHandler serves access tokens for the associated GCP service-account.
type TokenHandler struct {
	AllowedSources *net.IPNet
	TokenSource    oauth2.TokenSource
	Clock          func() time.Time
	robotAuth      auth
	k8s            *kubernetes.Clientset
}

type auth interface {
	CreateRobotTokenSource(context.Context) oauth2.TokenSource
	projectID() string
	robotName() string
}

type rAuth struct {
	robotauth.RobotAuth
}

func (a rAuth) projectID() string {
	return a.ProjectId
}

func (a rAuth) robotName() string {
	return a.RobotName
}

type TokenResponse struct {
	AccessToken  string `json:"access_token"`
	ExpiresInSec int    `json:"expires_in"`
	TokenType    string `json:"token_type"`
}

func NewTokenHandler(ctx context.Context, k8s *kubernetes.Clientset) (*TokenHandler, error) {
	_, allowedSources, err := net.ParseCIDR(*sourceCidr)
	if err != nil {
		return nil, fmt.Errorf("invalid source CIDR %s: %w", *sourceCidr, err)
	}

	t := &TokenHandler{
		AllowedSources: allowedSources,
		Clock:          time.Now,
		k8s:            k8s,
	}
	if err := t.updateRobotAuth(); err != nil {
		return nil, err
	}
	t.updateRobotTokenSource(ctx)
	return t, nil
}

func (th *TokenHandler) updateRobotAuth() error {
	robotAuth, err := robotauth.LoadFromFile(*robotIdFile)
	if err != nil {
		return fmt.Errorf("failed to read robot id file %s: %w", *robotIdFile, err)
	}
	th.robotAuth = &rAuth{*robotAuth}
	return nil
}

func (th *TokenHandler) updateRobotTokenSource(ctx context.Context) {
	th.TokenSource = newRateLimitTokenSource(th.robotAuth.CreateRobotTokenSource(ctx))
}

func (th *TokenHandler) NewMetadataHandler(ctx context.Context) *MetadataHandler {
	idHash := fnv.New64a()
	idHash.Write([]byte(th.robotAuth.robotName()))

	var projectNumber int64
	backoff.Retry(
		func() error {
			var err error
			projectNumber, err = getProjectNumber(oauth2.NewClient(ctx, th.TokenSource), th.robotAuth.projectID())
			if err != nil {
				log.Printf("will retry to obtain project number for %s: %v", th.robotAuth.projectID(), err)
			}
			return err
		},
		backoff.NewConstantBackOff(5*time.Second),
	)
	return &MetadataHandler{
		ClusterName:   th.robotAuth.robotName(),
		ProjectId:     th.robotAuth.projectID(),
		ProjectNumber: projectNumber,
		RobotName:     th.robotAuth.robotName(),
		InstanceId:    idHash.Sum64(),
		// This needs to be an actual Cloud zone so that it can be mapped
		// to a Monarch/Stackdriver region. TODO(swolter): We should make
		// this zone configurable to avoid confusing users.
		Zone: "europe-west1-c",
	}
}

// Return an access token for the 'robot-service' service account
// The query might also contain a 'scopes' query param, which we currently don't handle
// (e.g.: scopes=https://www.googleapis.com/auth/devstorage.full_control,https://www.googleapis.com/auth/cloud-platform HTTP/1.1)
func (th *TokenHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	ipPort := strings.Split(r.RemoteAddr, ":")
	if len(ipPort) != 2 {
		log.Printf("Unable to obtain IP from remote address %s", r.RemoteAddr)
		http.Error(w, "Unable to check authorization", http.StatusInternalServerError)
		return
	}
	if ip := net.ParseIP(ipPort[0]); ip == nil || !th.AllowedSources.Contains(ip) {
		log.Printf("Rejected remote IP %s", ipPort[0])
		http.Error(w, "Access forbidden", http.StatusForbidden)
		return
	}

	token, err := th.TokenSource.Token()
	if err != nil {
		log.Printf("Token retrieval error: %v", err)
		http.Error(w, fmt.Sprintf("Token retrieval failed: %v", err), http.StatusInternalServerError)
		return
	}

	now := th.Clock()
	expiresInSec := int(token.Expiry.Sub(now).Seconds())
	// fluent-bit expects expires_in - 10% > 60 seconds
	if expiresInSec < *minTokenExpiry {
		th.updateRobotTokenSource(r.Context())
		token, err = th.TokenSource.Token()
		if err != nil {
			log.Printf("Token retrieval error: %v", err)
			http.Error(w, fmt.Sprintf("Token retrieval failed: %v", err), http.StatusInternalServerError)
			return
		}
		expiresInSec = int(token.Expiry.Sub(now).Seconds())
	}

	tokenResponse := TokenResponse{
		AccessToken:  token.AccessToken,
		ExpiresInSec: expiresInSec,
		TokenType:    token.TokenType,
	}
	bytes, err := json.Marshal(tokenResponse)
	if err != nil {
		log.Printf("Token serialization error: %v", err)
		http.Error(w, fmt.Sprintf("Token serialization failed: %v", err), http.StatusInternalServerError)
		return
	}

	// Collect some extra data for diagnostics (aka which services rely on ADCs).
	// User-Agent: "Fluent-Bit", "gcloud-golang/0.1"
	ua := r.Header.Get("User-Agent")
	pod := th.getPodNameByIP(r.Context(), ipPort[0])

	w.Header().Set("Metadata-Flavor", "Google")
	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(http.StatusOK)
	w.Write(bytes)
	log.Printf("Served access token to %s (ua=%q, pod=%q)", r.RemoteAddr, ua, pod)
}

func (th *TokenHandler) getPodNameByIP(ctx context.Context, ip string) string {
	if th.k8s == nil || !*logPeerDetails {
		// TODO(ensonic): need to add a k8s fake to the tests
		return ""
	}

	// Meassure the time it takes to obtain the extra information
	defer func(start time.Time) {
		log.Printf("getPodNameByIP() took %s", time.Since(start))
	}(time.Now())

	// TODO(ensonic): to avoid traversing all ns/pods each time we can
	// - cache ip->ns/pod mapping
	// - check first if we can still get a pod by these keys and if the IP still matches
	// - do the listing otherwise
	// TODO(ensonic): consider labeling namespaces that participate in ADCs. This will speedup the lookups
	// and allows us to lock this down.

	podsToRetry := []corev1.Pod{}

	nss, err := th.k8s.CoreV1().Namespaces().List(ctx, metav1.ListOptions{})
	if err != nil {
		log.Printf("Failed to list namespaces: %v", err)
	}
	for _, ns := range nss.Items {
		nsName := ns.ObjectMeta.Name
		pods, err := th.k8s.CoreV1().Pods(nsName).List(ctx, metav1.ListOptions{})
		if err != nil {
			log.Printf("Failed to list pods in ns=%q: %v", nsName, err)
		}
		for _, pod := range pods.Items {
			if pod.Status.PodIP == ip {
				return nsName + "/" + pod.Name
			}
			if pod.Status.PodIP == "" {
				log.Printf("Pod %q has no ip (yet): %q", pod.Name, pod.Status.Message)
				podsToRetry = append(podsToRetry, pod)
			}
		}
	}
	// We don't have the resource version from the pod creation (to be used in the ListOptions above). Hence
	// we need to do this retry logic for cases where a pod just started and right away asked for an ADC.
	retries := getPodByIPRetries
	for len(podsToRetry) > 0 && retries > 0 {
		time.Sleep(getPodByIPWait)
		log.Printf("Retrying %d pods without ip", len(podsToRetry))

		ptr := podsToRetry
		podsToRetry = []corev1.Pod{}
		for _, p := range ptr {
			nsName := p.ObjectMeta.Namespace
			podName := p.ObjectMeta.Name
			pod, err := th.k8s.CoreV1().Pods(nsName).Get(ctx, podName, metav1.GetOptions{})
			if err != nil {
				log.Printf("Failed to get pod %q in ns=%q: %v", podName, nsName, err)
			}
			if pod.Status.PodIP == ip {
				return nsName + "/" + pod.Name
			}
			if pod.Status.PodIP == "" {
				log.Printf("Pod %q has no ip (yet): %q", pod.Name, pod.Status.Message)
				podsToRetry = append(podsToRetry, *pod)
			}
		}
		retries--
	}
	log.Printf("No pod found for ip=%q", ip)
	return ""
}

// ServiceAccountHandler serves information about the default service account.
type ServiceAccountHandler struct {
}

type ServiceAccountResponse struct {
	Aliases []string `json:"aliases"`
	Email   string   `json:"email"`
	Scopes  []string `json:"scopes"`
}

func (sh ServiceAccountHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	serviceAccountResponse := ServiceAccountResponse{
		Aliases: []string{},
		Email:   "default",
		Scopes:  []string{},
	}

	bytes, err := json.Marshal(serviceAccountResponse)
	if err != nil {
		log.Printf("ServiceAccountResponse serialization error: %v", err)
		http.Error(w, fmt.Sprintf("ServiceAccountResponse serialization failed: %v", err), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Metadata-Flavor", "Google")
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write(bytes)
	log.Printf("Responded to service-account request from %s for %s", r.RemoteAddr, r.URL.Path)
}

// MetadataHandler serves generic instance metadata.
type MetadataHandler struct {
	ClusterName   string
	ProjectId     string
	ProjectNumber int64
	RobotName     string
	InstanceId    uint64
	Zone          string
}

func (mh MetadataHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Metadata-Flavor", "Google")
	w.Header().Set("Content-Type", "application/text")

	metadata := map[string]string{
		"project/project-id":                   mh.ProjectId,
		"project/numeric-project-id":           fmt.Sprintf("%d", mh.ProjectNumber),
		"instance/hostname":                    fmt.Sprintf("robot-%s", mh.RobotName),
		"instance/id":                          fmt.Sprintf("%d", mh.InstanceId),
		"instance/zone":                        fmt.Sprintf("projects/%d/zones/%s", mh.ProjectNumber, mh.Zone),
		"instance/attributes/":                 "kube-env\ncluster-name\ncluster-location\n",
		"instance/attributes/kube-env":         fmt.Sprintf("CLUSTER_NAME: %s\n", mh.ClusterName),
		"instance/attributes/cluster-name":     mh.ClusterName,
		"instance/attributes/cluster-location": mh.Zone,
	}

	key := strings.TrimPrefix(r.URL.Path, "/computeMetadata/v1/")
	value := metadata[key]
	if value == "" {
		log.Printf("No key found for %s", r.URL.Path)
		http.NotFound(w, r)
		return
	}

	w.WriteHeader(http.StatusOK)
	w.Write([]byte(value))
	log.Printf("Responded to metadata request for %s: %s", r.URL.Path, value)
}

func getProjectNumber(client *http.Client, projectId string) (int64, error) {
	crm, err := cloudresourcemanager.New(client)
	if err != nil {
		return 0, err
	}
	project, err := crm.Projects.Get(projectId).Do()
	if err != nil {
		return 0, err
	}
	return project.ProjectNumber, nil
}
