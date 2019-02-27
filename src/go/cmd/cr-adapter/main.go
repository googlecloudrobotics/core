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

package main

import (
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"net"
	"regexp"
	"strconv"

	"github.com/golang/protobuf/jsonpb"
	"github.com/golang/protobuf/protoc-gen-go/descriptor"
	"github.com/jhump/protoreflect/dynamic"
	"google.golang.org/grpc"
	crdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset"
	_ "k8s.io/client-go/plugin/pkg/client/auth/gcp"
	"k8s.io/client-go/tools/clientcmd"
)

type K8sRequestParams struct {
	InMessageName        string
	OutMessageName       string
	Verb                 string
	OptionsAsQueryParams bool
	NameInPath           bool
	SetKindAndApiGroup   bool
	BodyFieldName        string
}

var (
	methodRE               = regexp.MustCompile("^/(.*)\\.K8s([^.]*)/([^/]*)$")
	resourceInfoRepository *ResourceInfoRepository
	messageFactory         = dynamic.NewMessageFactoryWithDefaults()
	// Global options for unmarshaling JSON to proto messages.
	// Allow unknown fields as a safety measure (in case the Kubernetes API server's version does
	// not match the version used to generate the proto descriptor).
	unmarshaler = &jsonpb.Unmarshaler{AllowUnknownFields: true}
)

func streamHandler(srv interface{}, stream grpc.ServerStream) error {
	fullMethodName, _ := grpc.MethodFromServerStream(stream)

	streamPkg, streamKind, method, err := parseMethodName(fullMethodName)
	if err != nil {
		return fmt.Errorf("unable to parse method name: %v", err)
	}

	resourceInfo, err := resourceInfoRepository.Lookup(streamPkg, streamKind)
	if err != nil {
		return err
	}

	var params *K8sRequestParams

	switch method {
	case "Get":
		params = &K8sRequestParams{
			InMessageName:        fmt.Sprintf("%s.Get%sRequest", streamPkg, streamKind),
			OutMessageName:       fmt.Sprintf("%s.%s", streamPkg, streamKind),
			Verb:                 "GET",
			OptionsAsQueryParams: true,
			NameInPath:           true,
			SetKindAndApiGroup:   false,
			BodyFieldName:        "",
		}
	case "List":
		params = &K8sRequestParams{
			InMessageName:        fmt.Sprintf("%s.List%sRequest", streamPkg, streamKind),
			OutMessageName:       fmt.Sprintf("%s.%sList", streamPkg, streamKind),
			Verb:                 "GET",
			OptionsAsQueryParams: true,
			NameInPath:           false,
			SetKindAndApiGroup:   false,
			BodyFieldName:        "",
		}
	case "Watch":
		params = &K8sRequestParams{
			InMessageName:        fmt.Sprintf("%s.Watch%sRequest", streamPkg, streamKind),
			OutMessageName:       fmt.Sprintf("%s.%sEvent", streamPkg, streamKind),
			Verb:                 "GET",
			OptionsAsQueryParams: true,
			NameInPath:           false,
			BodyFieldName:        "",
		}
	case "Create":
		params = &K8sRequestParams{
			InMessageName:        fmt.Sprintf("%s.Create%sRequest", streamPkg, streamKind),
			OutMessageName:       fmt.Sprintf("%s.%s", streamPkg, streamKind),
			Verb:                 "POST",
			OptionsAsQueryParams: true,
			NameInPath:           false,
			SetKindAndApiGroup:   true,
			BodyFieldName:        "object",
		}
	case "Update":
		params = &K8sRequestParams{
			InMessageName:        fmt.Sprintf("%s.Update%sRequest", streamPkg, streamKind),
			OutMessageName:       fmt.Sprintf("%s.%s", streamPkg, streamKind),
			Verb:                 "PUT",
			OptionsAsQueryParams: true,
			NameInPath:           true,
			SetKindAndApiGroup:   true,
			BodyFieldName:        "object",
		}
	case "UpdateStatus":
		return errors.New("UpdateStatus is not yet implemented")
	case "Delete":
		params = &K8sRequestParams{
			InMessageName:        fmt.Sprintf("%s.Delete%sRequest", streamPkg, streamKind),
			OutMessageName:       fmt.Sprintf("%s.Delete%sResponse", streamPkg, streamKind),
			Verb:                 "DELETE",
			OptionsAsQueryParams: false,
			NameInPath:           true,
			SetKindAndApiGroup:   false,
			BodyFieldName:        "options",
		}
	default:
		return fmt.Errorf("unsupported method: %v", method)
	}

	if method == "Watch" {
		err = streamingCall(stream, params, resourceInfo)
	} else {
		err = unaryCall(stream, params, resourceInfo)
	}
	if err != nil {
		log.Print(err)
	}
	return err
}

func unaryCall(stream grpc.ServerStream, params *K8sRequestParams, resource *ResourceInfo) error {

	inMessage, err := resource.GetMessage(params.InMessageName)
	if err != nil {
		return err
	}

	// Receive proto message.
	if err := stream.RecvMsg(inMessage); err != nil {
		return fmt.Errorf("error receiving message: %v", err)
	}

	// Add Kind and ApiGroup if necessary.
	if params.SetKindAndApiGroup {
		objectInterface, err := inMessage.TryGetFieldByName("object")
		if err != nil {
			return errors.New("unknown field: object")
		}
		object, ok := objectInterface.(*dynamic.Message)
		if !ok {
			return errors.New("object is not a message")
		}
		if err := object.TrySetFieldByName("kind", resource.Kind); err != nil {
			return errors.New("object has no kind field")
		}
		if err := object.TrySetFieldByName("apiVersion", resource.APIVersion); err != nil {
			return errors.New("object has no APIVersion field")
		}
	}

	// Create Kubernetes request.
	req := resource.Client.Verb(params.Verb).Resource(resource.KindPlural).Namespace("default")
	// Set resource name.
	var name string
	if params.NameInPath {
		name, err = getName(inMessage)
		if err != nil {
			return fmt.Errorf("unable to determine resource name: %v", err)
		}
		req = req.Name(name)
	}
	// Set query params.
	if params.OptionsAsQueryParams {
		queryParams, err := getQueryParams(inMessage)
		if err != nil {
			return fmt.Errorf("error determining query parameters: %v", err)
		}
		for k, v := range queryParams {
			req = req.Param(k, v)
		}
	}
	// Set body.
	if params.BodyFieldName != "" {
		body, err := getBody(inMessage, params.BodyFieldName)
		if err != nil {
			return fmt.Errorf("unable to create request body: %v", err)
		}
		req = req.Body(body)
	}

	// Perform Kubernetes request.
	if name == "" {
		log.Printf("Performing %s request for %s to %s", params.Verb, resource.KindPlural, req.URL().String())
	} else {
		log.Printf("Performing %s request for %s '%s' to %s", params.Verb, resource.Kind, name, req.URL().String())
	}
	res, err := req.DoRaw()
	if err != nil {
		return fmt.Errorf("Kubernetes request failed: %v. Response body: %s", err, res)
	}

	// Create instance of dynamic message for received data.
	outMessage, err := resource.GetMessage(params.OutMessageName)
	if err != nil {
		return err
	}

	// Unmarshal kubernetes response to proto message.
	if err = outMessage.UnmarshalJSONPB(unmarshaler, res); err != nil {
		return fmt.Errorf("error unmarshaling response from Kubernetes: %v\n%s", err, string(res))
	}

	// Send proto message.
	if err = stream.SendMsg(outMessage); err != nil {
		return fmt.Errorf("error sending message: %v", err)
	}

	return nil
}

func streamingCall(stream grpc.ServerStream, params *K8sRequestParams, resource *ResourceInfo) error {

	// Create dynamic proto message instances for i/o.
	inMessage, err := resource.GetMessage(params.InMessageName)
	if err != nil {
		return err
	}
	outMessage, err := resource.GetMessage(params.OutMessageName)
	if err != nil {
		return err
	}

	// Receive proto message.
	if err := stream.RecvMsg(inMessage); err != nil {
		return fmt.Errorf("error receiving message: %v", err)
	}

	// Create kubernetes request.
	req := resource.Client.Verb(params.Verb).Resource(resource.KindPlural).Namespace("default")
	// Set query params.
	queryParams, err := getQueryParams(inMessage)
	if err != nil {
		return fmt.Errorf("error determining query parameters: %v", err)
	}
	for k, v := range queryParams {
		if k != "watch" {
			req = req.Param(k, v)
		}
	}
	req = req.Param("watch", "true")

	// Send kubernetes request.
	log.Printf("Performing %s request for %s to %s", params.Verb, resource.KindPlural, req.URL().String())
	str, err := req.Stream()
	if err != nil {
		return fmt.Errorf("kubernetes request failed: %v", err)
	}
	defer str.Close()

	// Process response stream.
	dec := json.NewDecoder(str)
	for {
		if err := unmarshaler.UnmarshalNext(dec, outMessage); err != nil {
			if err == io.EOF {
				return nil
			}
			return fmt.Errorf("error unmarshaling response from kubernetes: %v", err)
		}
		if err = stream.SendMsg(outMessage); err != nil {
			return fmt.Errorf("error sending message: %v", err)
		}
		outMessage.Reset()
	}
}

func getName(message *dynamic.Message) (string, error) {
	// Try field name "name".
	nameInterface, err := message.TryGetFieldByName("name")
	if err != nil {
		// Try field name "object".
		objInterface, err := message.TryGetFieldByName("object")
		if err != nil {
			return "", fmt.Errorf("unable to extract field 'name' or 'object': %v", err)
		}
		objMessage, ok := objInterface.(*dynamic.Message)
		if !ok {
			return "", errors.New("object is not a message")
		}
		metaInterface, err := objMessage.TryGetFieldByName("metadata")
		if err != nil {
			return "", fmt.Errorf("unable to extract field metadata: %v", err)
		}
		metaMessage, ok := metaInterface.(*dynamic.Message)
		if !ok {
			return "", errors.New("metadata is not a message")
		}
		nameInterface, err = metaMessage.TryGetFieldByName("name")
		if err != nil {
			return "", fmt.Errorf("unable to extract resource name: %v", err)
		}
	}
	name, ok := nameInterface.(string)
	if !ok {
		return "", errors.New("name is not a string")
	}
	return name, nil
}

func getBody(message *dynamic.Message, fieldName string) ([]byte, error) {
	obj, err := message.TryGetFieldByName(fieldName)
	if err != nil {
		return nil, fmt.Errorf("unable to extract field %s: %v", fieldName, err)
	}
	bodyMessage, ok := obj.(*dynamic.Message)
	if !ok {
		return nil, fmt.Errorf("%s is not a message", fieldName)
	}
	body, err := bodyMessage.MarshalJSON()
	if err != nil {
		return nil, fmt.Errorf("unable to marshal body message to JSON: %v", err)
	}
	return body, nil
}

func getQueryParams(message *dynamic.Message) (map[string]string, error) {
	optionsInterface, err := message.TryGetFieldByName("options")
	if err != nil {
		return nil, errors.New("unknown field: options")
	}
	options, ok := optionsInterface.(*dynamic.Message)
	if !ok {
		return nil, errors.New("options is not a message")
	}
	queryParams := make(map[string]string)
	if options != nil {
		optionFields := options.GetKnownFields()
		for _, f := range optionFields {
			if options.HasField(f) {
				optionName := f.GetName()
				optionType := f.GetType()
				optionValueInterface := options.GetField(f)
				var optionValue string
				ok = false
				switch optionType {
				case descriptor.FieldDescriptorProto_TYPE_STRING:
					// TODO(daschmidt): Do we need to handle repeated string fields?
					// (Case in point: UpdateOptions.dryRun)
					optionValue, ok = optionValueInterface.(string)
				case descriptor.FieldDescriptorProto_TYPE_INT64:
					var optionValueInt64 int64
					optionValueInt64, ok = optionValueInterface.(int64)
					optionValue = strconv.FormatInt(optionValueInt64, 10)
				case descriptor.FieldDescriptorProto_TYPE_BOOL:
					var optionValueBool bool
					optionValueBool, ok = optionValueInterface.(bool)
					optionValue = strconv.FormatBool(optionValueBool)
				}
				if ok {
					queryParams[optionName] = optionValue
				} else {
					log.Printf("unable to process option field %s.", optionName)
				}
			}
		}
	}
	return queryParams, nil
}

func parseMethodName(methodName string) (streamPkg string, streamKind string, method string, err error) {
	matches := methodRE.FindStringSubmatch(methodName)
	if len(matches) != 4 {
		err = fmt.Errorf("expected 4 matches in %s", methodName)
		return
	}
	streamPkg = matches[1]
	streamKind = matches[2]
	method = matches[3]
	return
}

func main() {
	var kubeconfig string
	var master string
	var port int

	flag.StringVar(&kubeconfig, "k", "", "absolute path to the kubeconfig file")
	flag.IntVar(&port, "p", 50053, "listening port of gRPC server")
	flag.StringVar(&master, "m", "", "master URL")

	flag.Parse()

	// Build kubeconfig.
	config, err := clientcmd.BuildConfigFromFlags(master, kubeconfig)
	if err != nil {
		log.Fatalf("error building Kubernetes config: %v", err)
	}

	resourceInfoRepository = NewResourceInfoRepository(config)
	go func() {
		done := make(chan struct{})
		clientset, err := crdclientset.NewForConfig(config)
		if err != nil {
			log.Fatalf("error building CRD clientset: %v", err)
		}
		if err := resourceInfoRepository.Update(done, clientset); err != nil {
			log.Fatalf("error building resource info repository: %v", err)
		}
	}()
	go func() {
		for {
			select {
			case err := <-resourceInfoRepository.ErrorChannel():
				log.Printf("error in CR definition: %v", err)
			case msg := <-resourceInfoRepository.LogChannel():
				log.Print(msg)
			}
		}
	}()

	// Start gRPC server.
	lis, err := net.Listen("tcp", fmt.Sprintf("localhost:%d", port))
	if err != nil {
		log.Fatalf("failed to listen: %v", err)
	}
	grpcServer := grpc.NewServer(grpc.UnknownServiceHandler(streamHandler))
	log.Fatal(grpcServer.Serve(lis))
}
