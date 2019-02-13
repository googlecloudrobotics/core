package main

import (
	"bytes"
	"encoding/base64"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"strings"

	apiextensions "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	"k8s.io/apimachinery/pkg/runtime/serializer/json"
)

var (
	protoDescriptorFile = flag.String("proto_descriptor_file", "",
		"Filename with a binary FileDescriptorSet")
	message    = flag.String("message", "", "Fully qualified proto message of the CRD")
	plural     = flag.String("plural", "", "Plural form of the message")
	group      = flag.String("group", "", "Kubernetes API group")
	namespaced = flag.Bool("namespaced", true, "Generated CR should be namespaced")
	output     = flag.String("output", "", "Output file")
)

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

	crd := apiextensions.CustomResourceDefinition{}
	crd.TypeMeta.APIVersion = apiextensions.SchemeGroupVersion.String()
	crd.TypeMeta.Kind = "CustomResourceDefinition"
	crd.ObjectMeta.Name = fmt.Sprintf("%s.%s", pluralName, *group)
	crd.ObjectMeta.Annotations = map[string]string{"crc.cloudrobotics.com/proto-descriptor": base64.StdEncoding.EncodeToString(descriptor)}
	crd.Spec.Group = *group
	crd.Spec.Names.Plural = pluralName
	crd.Spec.Names.Kind = kind
	crd.Spec.Scope = apiextensions.NamespaceScoped
	crd.Spec.Versions = []apiextensions.CustomResourceDefinitionVersion{{
		Name:    version,
		Storage: true,
		Served:  true,
	}}
	crd.Status.Conditions = []apiextensions.CustomResourceDefinitionCondition{}
	crd.Status.StoredVersions = []string{version}

	buf := new(bytes.Buffer)
	serializer := json.NewYAMLSerializer(json.DefaultMetaFactory, nil, nil)
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
