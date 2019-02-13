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

package util

import (
	"fmt"
	"net"
	"strconv"
	"strings"
)

type Factory interface {
	ScanInt() (int, error)
	GetNetworkInterfaceIP(string) (string, error)
}

type DefaultFactory struct{}

func NewFactory() *DefaultFactory {
	return &DefaultFactory{}
}

// Read stdin up to the next space and convert to an int.
func (f *DefaultFactory) ScanInt() (int, error) {
	var s string
	_, err := fmt.Scan(&s)
	if err != nil {
		return 0, err
	}

	i, err := strconv.Atoi(s)
	if err != nil {
		return 0, err
	}
	return i, nil
}

// GetNetworkInterfaceIP returns the IP address of the first local network interface whose name
// starts with namePrefix
func (f *DefaultFactory) GetNetworkInterfaceIP(namePrefix string) (string, error) {
	ifaces, err := net.Interfaces()
	if err != nil {
		return "", err
	}
	for _, i := range ifaces {
		if strings.HasPrefix(i.Name, namePrefix) {
			addrs, err := i.Addrs()
			if err != nil {
				return "", err
			}
			for _, address := range addrs {
				if ipnet, ok := address.(*net.IPNet); ok {
					if ipnet.IP.To4() != nil {
						return ipnet.IP.String(), nil
					}
				}
			}
		}
	}
	return "", fmt.Errorf("Could not look up IP of interface with prefix: %v", namePrefix)
}
