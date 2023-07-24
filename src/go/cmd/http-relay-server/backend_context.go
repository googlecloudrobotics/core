package main

import "net/http"

type backendContext struct {
	Id         string
	ServerName string
	Path       string
}

func newBackendContext(r *http.Request) (*backendContext, error) {
	serverName, path, err := extractBackendNameAndPath(r)
	if err != nil {
		return nil, err
	}
	return &backendContext{
		Id:         serverName + ":" + createId(),
		ServerName: serverName,
		Path:       path,
	}, nil
}
