// request.go contains methods for understanding and validating the incoming
// request.
package main

import (
	"fmt"
	"net/http"
	"net/url"
	"regexp"
	"slices"
	"strings"
)

// Regex for RFC 1123 subdomain format
// https://kubernetes.io/docs/concepts/overview/working-with-objects/names/#dns-label-names
// https://github.com/kubernetes/kubernetes/blob/976a940f4a4e84fe814583848f97b9aafcdb083f/staging/src/k8s.io/apimachinery/pkg/util/validation/validation.go#L209
var isValidRobotName = regexp.MustCompile(`^[a-z0-9]([-a-z0-9]*[a-z0-9])?(\.[a-z0-9]([-a-z0-9]*[a-z0-9])?)*$`).MatchString

// the prefix of the label selector query param used by the cr-syncer
const robotNameSelectorPrefix = "cloudrobotics.com/robot-name="

// incomingRequest contains the authz-relevant properties of the resource
type incomingRequest struct {
	// GroupKind, eg "registry.cloudrobotics.com/robots"
	GroupKind string

	// RobotName, or empty if no label selector is used (eg for a Get or Update)
	RobotName string

	// ResourceName, or empty if no resource is specified (eg for a List or Watch of a filtered resource)
	ResourceName string
}

// extractOriginalURL returns the raw origin URL or path from proxy headers.
// Any scheme/host in X-Original-Url is stripped when passed to url.Parse(u).Path,
// normalizing it to match X-Envoy-Original-Path.
func extractOriginalURL(r *http.Request) string {
	if u := r.Header.Get("X-Original-Url"); u != "" {
		return u
	}
	return r.Header.Get("X-Envoy-Original-Path")
}

// parseURL parses the URL that the cr-syncer is hitting to find the
// authz-relevant properties.
func parseURL(urlString string) (*incomingRequest, error) {
	result := incomingRequest{}
	url, err := url.Parse(urlString)
	if err != nil {
		return nil, err
	}

	// Path should be one of:
	//  /apis/core.kubernetes/apis/<group>/<version>/<kind>
	//  /apis/core.kubernetes/apis/<group>/<version>/namespaces/<namespace>/<kind>
	//  /apis/core.kubernetes/apis/<group>/<version>/namespaces/<namespace>/<kind>/<resourceName>
	//  /apis/core.kubernetes/apis/<group>/<version>/namespaces/<namespace>/<kind>/<resourceName>/status
	//                             parts[0] parts[1] parts[2]   parts[3]   parts[4] parts[5]
	parts := strings.Split(strings.TrimPrefix(url.Path, "/apis/core.kubernetes/apis/"), "/")
	if len(parts) < 3 || len(parts) > 7 {
		return nil, fmt.Errorf("unexpected URL path %q (split into %d path segments, expeced betwen 3 and 7)", url.Path, len(parts))
	}
	if parts[2] != "namespaces" {
		// Add in "/namespaces/default" so remaining code can use fixed indices.
		// I also considered a regexp but it's not pretty:
		// "/apis/core.kubernetes/apis/([^/]*)/([^/]*)(/namespaces/[^/]*)?/([^/]*)/?([^/]*)(/status)?"
		parts = slices.Insert(parts, 2, "namespaces", "default")
	}

	result.GroupKind = fmt.Sprintf("%s/%s", parts[0], parts[4])
	if len(parts) > 5 {
		// if a resourceName is in the URL, we don't need to look at the query parameters
		result.ResourceName = parts[5]
		return &result, nil
	}

	// If we have no resourceName, this is a list/watch request, so check for a
	// labelSelector.
	params := url.Query()
	labelSelectors := params["labelSelector"]
	if len(labelSelectors) == 0 {
		// This is an unfiltered List or Watch request (eg for robottypes).
		return &result, nil
	}
	if len(labelSelectors) > 1 || !strings.HasPrefix(labelSelectors[0], robotNameSelectorPrefix) {
		return nil, fmt.Errorf("invalid label selector %v", labelSelectors)
	}
	result.RobotName = strings.TrimPrefix(labelSelectors[0], robotNameSelectorPrefix)
	if !isValidRobotName(result.RobotName) {
		return nil, fmt.Errorf("invalid robot name %q", result.RobotName)
	}
	return &result, nil
}
