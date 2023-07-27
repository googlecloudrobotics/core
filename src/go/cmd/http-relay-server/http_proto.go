// Utilities for proto conversion
package main

import (
	"log"
	"net/http"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"google.golang.org/protobuf/proto"
)

func marshalHeader(h *http.Header) []*pb.HttpHeader {
	r := []*pb.HttpHeader{}
	for k, vs := range *h {
		for _, v := range vs {
			r = append(r, &pb.HttpHeader{Name: proto.String(k), Value: proto.String(v)})
		}
	}
	return r
}

func unmarshalHeader(w http.ResponseWriter, protoHeader []*pb.HttpHeader) {
	for _, h := range protoHeader {
		log.Printf("Writing header kv: %v = %v", *h.Name, *h.Value)
		w.Header().Add(*h.Name, *h.Value)
	}
}
