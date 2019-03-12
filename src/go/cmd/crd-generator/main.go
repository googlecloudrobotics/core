package main

import (
	"bytes"
	"encoding/base64"
	"encoding/json"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"strconv"
	"strings"

	"github.com/go-openapi/spec"
	apiextensions "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	k8sjson "k8s.io/apimachinery/pkg/runtime/serializer/json"
)

var (
	protoDescriptorFile = flag.String("proto_descriptor_file", "",
		"Filename with a binary FileDescriptorSet")
	message           = flag.String("message", "", "Fully qualified proto message of the CRD")
	swagger           = flag.String("openapi_spec_file", "", "Swagger doc for the generated API")
	plural            = flag.String("plural", "", "Plural form of the message")
	group             = flag.String("group", "", "Kubernetes API group")
	namespaced        = flag.Bool("namespaced", false, "Generated CR should be namespaced")
	output            = flag.String("output", "", "Output file")
	filterByRobotName = flag.Bool("filter_by_robot_name", false, "Annotation for the CR Syncer")
	specSource        = flag.String("spec_source", "cloud", "Annotation for the CR Syncer")
)

func deleteDefaults(input *map[string]interface{}) {
	delete(*input, "default")
	for _, value := range *input {
		if m, ok := value.(map[string]interface{}); ok {
			deleteDefaults(&m)
		}
	}
}

func hasReferences(input map[string]interface{}) bool {
	if _, ok := input["$ref"]; ok {
		return true
	}
	for _, value := range input {
		if m, ok := value.(map[string]interface{}); ok {
			if hasReferences(m) {
				return true
			}
		}
	}
	return false
}

func createValidation(swaggerFile string, version string, kind string) (*apiextensions.CustomResourceValidation, error) {
	swaggerJson, err := ioutil.ReadFile(swaggerFile)
	if err != nil {
		return nil, err
	}

	swagger := spec.Swagger{}
	if err := swagger.UnmarshalJSON(swaggerJson); err != nil {
		return nil, err
	}

	result := &apiextensions.CustomResourceValidation{
		OpenAPIV3Schema: &apiextensions.JSONSchemaProps{
			Properties: make(map[string]apiextensions.JSONSchemaProps),
		},
	}
	for _, field := range []string{"Spec", "Status"} {
		msg := fmt.Sprintf("%s%s%s", version, kind, field)
		schema, ok := swagger.SwaggerProps.Definitions[msg]
		if !ok {
			return nil, fmt.Errorf("Expected definition for %s in %s", msg, swaggerFile)
		}
		if err := spec.ExpandSchema(&schema, swagger.SwaggerProps, nil); err != nil {
			return nil, err
		}
		b, err := json.Marshal(schema)
		if err != nil {
			return nil, err
		}
		unstructured := make(map[string]interface{})
		if err := json.Unmarshal(b, &unstructured); err != nil {
			return nil, err
		}
		deleteDefaults(&unstructured)
		if hasReferences(unstructured) {
			return nil, fmt.Errorf("Kubernetes API server's validation does not support self-referencing protos")
		}
		b, err = json.Marshal(unstructured)
		if err != nil {
			return nil, err
		}
		props := apiextensions.JSONSchemaProps{}
		if err := json.Unmarshal(b, &props); err != nil {
			return nil, err
		}
		result.OpenAPIV3Schema.Properties[strings.ToLower(field)] = props
	}

	return result, nil
}

func main() {
	flag.Parse()

	messageParts := strings.Split(*message, ".")
	if len(messageParts) < 3 {
		log.Fatalf("Want fully qualified proto message name with at least 2 dots, got %s", *message)
	}
	kind := messageParts[len(messageParts)-1]
	version := messageParts[len(messageParts)-2]
	pluralName := *plural
	if pluralName == "" {
		pluralName = strings.ToLower(kind) + "s"
	}

	descriptor, err := ioutil.ReadFile(*protoDescriptorFile)
	if err != nil {
		log.Fatalf("Unable to read proto descriptor file %s: %v", *protoDescriptorFile, err)
	}

	validation, err := createValidation(*swagger, version, kind)
	if err != nil {
		log.Fatalf("Unable to convert Swagger definition to validation: %v", err)
	}

	crd := apiextensions.CustomResourceDefinition{}
	crd.TypeMeta.APIVersion = apiextensions.SchemeGroupVersion.String()
	crd.TypeMeta.Kind = "CustomResourceDefinition"
	crd.ObjectMeta.Name = fmt.Sprintf("%s.%s", pluralName, *group)
	crd.ObjectMeta.Annotations = map[string]string{
		"crc.cloudrobotics.com/proto-descriptor":           base64.StdEncoding.EncodeToString(descriptor),
		"cr-syncer.cloudrobotics.com/filter-by-robot-name": strconv.FormatBool(*filterByRobotName),
		"cr-syncer.cloudrobotics.com/spec-source":          *specSource,
	}
	crd.Spec.Group = *group
	crd.Spec.Names.Plural = pluralName
	crd.Spec.Names.Kind = kind
	crd.Spec.Scope = apiextensions.NamespaceScoped
	crd.Spec.Versions = []apiextensions.CustomResourceDefinitionVersion{{
		Name:    version,
		Storage: true,
		Served:  true,
	}}
	crd.Spec.Validation = validation
	crd.Status.Conditions = []apiextensions.CustomResourceDefinitionCondition{}
	crd.Status.StoredVersions = []string{version}

	buf := new(bytes.Buffer)
	serializer := k8sjson.NewYAMLSerializer(k8sjson.DefaultMetaFactory, nil, nil)
	if err := serializer.Encode(&crd, buf); err != nil {
		log.Fatalf("Unable to serialize CRD: %v", err)
	}
	if *output != "" {
		if err := ioutil.WriteFile(*output, buf.Bytes(), 0644); err != nil {
			log.Fatalf("Unable to write %s: %v", *output, err)
		}
	} else {
		os.Stdout.Write(buf.Bytes())
	}
}
