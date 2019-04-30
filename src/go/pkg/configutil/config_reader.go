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

package configutil

import (
	"bufio"
	"context"
	"io"
	"regexp"
	"strings"

	"cloud.google.com/go/storage"
)

// Unescapes a bash string. Only supports the following three patterns:
//   Hello\ \w\o\r\l\d -> Hello world
//   "Hello \"world\"" -> Hello "world"
//   'Hello '\''world'\''' -> Hello 'world'
func bashUnescape(s string) string {
	if len(s) <= 1 {
		return s
	}
	if s[0] == '\'' && s[len(s)-1] == '\'' {
		return strings.ReplaceAll(s[1:len(s)-1], `'\''`, `'`)
	}
	if s[0] == '"' && s[len(s)-1] == '"' {
		s = s[1 : len(s)-1]
		// Unescape \\, \$, \", \`, and \!.
		re := regexp.MustCompile("\\\\([\\$\"`!\\\\])")
		return re.ReplaceAllString(s, "$1")
	}
	re := regexp.MustCompile(`\\(.)`)
	return re.ReplaceAllString(s, "$1")
}

func getConfigFromReader(reader io.Reader) (map[string]string, error) {
	re := regexp.MustCompile(`^\s*([\w]*)=(.*?)\s*$`)
	s := bufio.NewScanner(reader)
	vars := make(map[string]string)
	for s.Scan() {
		match := re.FindStringSubmatch(s.Text())
		if len(match) >= 3 {
			// TODO(skopecki) Consider allowing variable substitution (e.g., FOOBAR="${FOO}/bar")
			vars[match[1]] = bashUnescape(match[2])
		}
	}
	if err := s.Err(); err != nil {
		return nil, err
	}
	return vars, nil
}

func setDefaultVars(vars map[string]string) {
	// Set defaults values for optional variables.
	if vars["CLOUD_ROBOTICS_CONTAINER_REGISTRY"] == "" {
		vars["CLOUD_ROBOTICS_CONTAINER_REGISTRY"] = "gcr.io/" + vars["GCP_PROJECT_ID"]
	}
}

// ReadConfig reads the config.sh from the cloud storage of the given project.
// All variables specified in the config are returned as dictionary.
// Uses the following defaults if the variables are not set:
//   CLOUD_ROBOTICS_CONTAINER_REGISTRY="gcr.io/<GCP_PROJECT_ID>"
func ReadConfig(project string) (map[string]string, error) {
	ctx := context.Background()
	client, err := storage.NewClient(ctx)
	if err != nil {
		return nil, err
	}

	bkt := client.Bucket(project + "-cloud-robotics-config")
	reader, err := bkt.Object("config.sh").NewReader(ctx)
	if err != nil {
		return nil, err
	}
	defer reader.Close()

	vars, err := getConfigFromReader(reader)
	if err != nil {
		return nil, err
	}
	setDefaultVars(vars)
	return vars, nil
}
