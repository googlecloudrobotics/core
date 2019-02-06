// Copyright 2019 The Google Cloud Robotics Authors
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
	"encoding/json"
	"flag"
	"fmt"
	"hash/fnv"
	"io/ioutil"
	"log"
	"net"
	"net/http"
	"os"
	"os/exec"
	"os/signal"
	"strconv"
	"strings"
	"syscall"
	"time"

	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"

	"github.com/fsnotify/fsnotify"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"golang.org/x/oauth2"
	cloudresourcemanager "google.golang.org/api/cloudresourcemanager/v1"
)

var (
	bindIP      string
	port        = flag.Int("port", 80, "Port number to listen on")
	robotIdFile = flag.String("robot_id_file", "", "robot-id.json file")
	sourceCidr  = flag.String("source_cidr", "127.0.0.1/32", "CIDR giving allowed source addresses for token retrieval")
)

type TokenHandler struct {
	AllowedSources *net.IPNet
	TokenSource    oauth2.TokenSource
	Clock          func() time.Time
}

type TokenResponse struct {
	AccessToken  string `json:"access_token"`
	ExpiresInSec int    `json:"expires_in"`
	TokenType    string `json:"token_type"`
}

func (th TokenHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
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
	if now.After(token.Expiry) {
		http.Error(w, "Internal access token is expired", http.StatusInternalServerError)
		return
	}

	tokenResponse := TokenResponse{
		AccessToken:  token.AccessToken,
		ExpiresInSec: int(token.Expiry.Sub(now).Seconds()),
		TokenType:    token.TokenType,
	}
	bytes, err := json.Marshal(tokenResponse)
	if err != nil {
		log.Printf("Token serialization error: %v", err)
		http.Error(w, fmt.Sprintf("Token serialization failed: %v", err), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Metadata-Flavor", "Google")
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write(bytes)
	log.Printf("Served access token to %s", r.RemoteAddr)
}

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

func detectChangesToFile(filename string) <-chan struct{} {
	watcher, err := fsnotify.NewWatcher()
	if err != nil {
		log.Fatal(err)
	}
	err = watcher.Add(filename)
	if err != nil {
		log.Fatal(err)
	}
	changes := make(chan struct{})
	go func() {
		for {
			select {
			case event, ok := <-watcher.Events:
				if !ok {
					return
				}
				log.Printf("event on %s: %s", filename, event)
				if event.Op&(fsnotify.Write|fsnotify.Remove) != 0 {
					changes <- struct{}{}
				}
			case err, ok := <-watcher.Errors:
				if !ok {
					return
				}
				log.Println("watcher error:", err)
			}
		}
	}()
	return changes
}

func addIPTablesRule() error {
	args := append([]string{"-t", "nat", "-A", "PREROUTING"}, getIPTablesRuleSpec()...)
	if err := runIPTablesCommand(args); err != nil {
		return fmt.Errorf("iptables invocation failed: %v", err)
	}
	return nil
}

func removeIPTablesRule() {
	args := append([]string{"-t", "nat", "-D", "PREROUTING"}, getIPTablesRuleSpec()...)
	if err := runIPTablesCommand(args); err != nil {
		log.Printf("Warning: iptables invocation failed: %v", err)
	}
}

func getIPTablesRuleSpec() []string {
	return []string{
		"-p", "tcp",
		"-d", "169.254.169.254",
		"--dport", "80",
		"-j", "DNAT",
		"--to-destination", bindIP + ":" + strconv.Itoa(*port),
	}
}

func runIPTablesCommand(args []string) error {
	log.Printf("Debug: Running iptables command with args %v", args)
	cmd := exec.Command("iptables", args...)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	return cmd.Run()
}

func patchCorefileInCluster() error {
	config, err := rest.InClusterConfig()
	if err != nil {
		return err
	}
	k8s, err := kubernetes.NewForConfig(config)
	if err != nil {
		return err
	}
	return PatchCorefile(k8s)
}

func revertCorefileInCluster() {
	config, err := rest.InClusterConfig()
	if err != nil {
		log.Printf("Warning: failed to create InClusterConfig: %v", err)
		return
	}
	k8s, err := kubernetes.NewForConfig(config)
	if err != nil {
		log.Printf("Warning: failed to create ClientSet: %v", err)
		return
	}
	if err := RevertCorefile(k8s); err != nil {
		log.Printf("Warning: failed to revert Corefile: %v", err)
	}
}

func main() {
	flag.Parse()

	bindIP = os.Getenv("BIND_IP")
	if bindIP == "" {
		log.Fatal("BIND_IP environment variable not set")
	}

	_, allowedSources, err := net.ParseCIDR(*sourceCidr)
	if err != nil {
		log.Fatalf("invalid source CIDR %s: %v", *sourceCidr, err)
	}

	robotIdBytes, err := ioutil.ReadFile(*robotIdFile)
	if err != nil {
		log.Fatalf("failed to read robot id file %s: %v", *robotIdFile, err)
	}

	var robotAuth robotauth.RobotAuth
	err = json.Unmarshal(robotIdBytes, &robotAuth)
	if err != nil {
		log.Fatalf("failed to parse robot id file %s: %v", *robotIdFile, err)
	}

	idHash := fnv.New64a()
	idHash.Write([]byte(robotAuth.RobotName))

	ctx := context.Background()
	tokenSource := robotAuth.CreateRobotTokenSource(ctx)
	var projectNumber int64
	for {
		projectNumber, err = getProjectNumber(oauth2.NewClient(ctx, tokenSource), robotAuth.ProjectId)
		if err != nil {
			log.Printf("will retry to obtain project number for %s: %v", robotAuth.ProjectId, err)
			time.Sleep(5 * time.Second)
		} else {
			break
		}
	}
	tokenHandler := TokenHandler{
		AllowedSources: allowedSources,
		TokenSource:    tokenSource,
		Clock:          time.Now,
	}
	http.Handle("/computeMetadata/v1/instance/service-accounts/default/token", tokenHandler)
	serviceAccountHandler := ServiceAccountHandler{}
	http.Handle("/computeMetadata/v1/instance/service-accounts/default/", serviceAccountHandler)
	metadataHandler := MetadataHandler{
		ClusterName:   "robot-robotics",
		ProjectId:     robotAuth.ProjectId,
		ProjectNumber: projectNumber,
		RobotName:     robotAuth.RobotName,
		InstanceId:    idHash.Sum64(),
		// This needs to be an actual Cloud zone so that it can be mapped
		// to a Monarch/Stackdriver region. TODO(swolter): We should make
		// this zone configurable to avoid confusing users.
		Zone: "europe-west1-c",
	}
	http.Handle("/computeMetadata/v1/", metadataHandler)
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/" {
			log.Printf("Handling root request for %s", r.URL.Path)
			w.Header().Set("Metadata-Flavor", "Google")
			w.Header().Set("Content-Type", "application/text")
			w.WriteHeader(http.StatusOK)
			w.Write([]byte("computeMetadata/\n"))
		} else {
			log.Printf("Unhandled request %s from %s", r.URL.Path, r.RemoteAddr)
			http.NotFound(w, r)
		}
	})

	// Make sure that the bind is successful before adding the iptables rule as a means of
	// avoiding a race condition where an old metadata-server instance that has not yet fully
	// terminated removes the iptables rule again.
	bindAddress := fmt.Sprintf("%s:%d", bindIP, *port)
	ln, err := net.Listen("tcp", bindAddress)
	if err != nil {
		log.Fatalf("failed to create listener on %s: %v", bindAddress, err)
	}
	log.Printf("Listening on %s", bindAddress)

	if err := addIPTablesRule(); err != nil {
		log.Fatalf("failed to add iptables rule: %v", err)
	}

	if err := patchCorefileInCluster(); err != nil {
		removeIPTablesRule()
		log.Fatal(err)
	}

	go func() {
		err = http.Serve(ln, nil)
		revertCorefileInCluster()
		removeIPTablesRule()
		log.Fatal(err)
	}()

	go func() {
		<-detectChangesToFile(*robotIdFile)
		revertCorefileInCluster()
		removeIPTablesRule()
		log.Fatalf("%s changed but reloading is not implemented. Crashing...", *robotIdFile)
	}()

	stop := make(chan os.Signal)
	signal.Notify(stop, syscall.SIGTERM)

	<-stop
	revertCorefileInCluster()
	removeIPTablesRule()
}
