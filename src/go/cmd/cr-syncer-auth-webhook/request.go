// request.go contains methods for understanding and validating the incoming
// request.
package main

import (
	"fmt"
	"net/url"
	"slices"
	"strings"

	"github.com/pkg/errors"
)

// incomingRequest contains the authz-relevant properties of the resource
type incomingRequest struct {
	// GroupKind, eg "registry.cloudrobotics.com/robots"
	GroupKind string

	// RobotName, or empty if no label selector is used (eg for a Get or Update)
	RobotName string

	// ResourceName, or empty if no resource is specified (eg for a List or Watch of a filtered resource)
	ResourceName string
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
		return nil, errors.New("unexpected URL length")
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
		return nil, errors.New("invalid label selector")
	}
	result.RobotName = strings.TrimPrefix(labelSelectors[0], robotNameSelectorPrefix)
	if !isValidRobotName(result.RobotName) {
		return nil, errors.New("invalid robot name")
	}
	return &result, nil
}
