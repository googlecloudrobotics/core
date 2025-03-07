// request.go contains methods for understanding and validating the in-flight
// request.
package main

import (
	"fmt"
	"net/url"
	"strings"

	"github.com/pkg/errors"
)

// analysis contains the authz-relevant properties of the resource
type analysis struct {
	// groupKind, eg "registry.cloudrobotics.com/robots"
	groupKind string

	// robotName, or empty if no label selector is used (eg for a Get or Update)
	robotName string

	// resourceID, or empty if no resource is specified (eg for a List or Watch of a filtered resource)
	resourceName string
}

func analyzeURL(urlString string) (*analysis, error) {
	result := analysis{}
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
		parts = append(append(parts[:2], "namespaces", "default"), parts[2:]...)
	}

	result.groupKind = fmt.Sprintf("%s/%s", parts[0], parts[4])
	if len(parts) > 5 {
		// if a resourceName is in the URL, we don't need to look at the query parameters
		result.resourceName = parts[5]
		return &result, nil
	}

	// no resourceName, look for a labelSelector
	params := url.Query()
	labelSelectors := params["labelSelector"]
	if len(labelSelectors) == 0 {
		// This is an unfiltered List or Watch request (eg for robottypes).
		return &result, nil
	}
	if len(labelSelectors) > 1 || !strings.HasPrefix(labelSelectors[0], robotNameSelectorPrefix) {
		return nil, errors.New("invalid label selector")
	}
	result.robotName = strings.TrimPrefix(labelSelectors[0], robotNameSelectorPrefix)
	if !isValidRobotName(result.robotName) {
		return nil, errors.New("invalid robot name")
	}
	return &result, nil
}
