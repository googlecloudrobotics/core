module github.com/googlecloudrobotics/core/src

go 1.17

require (
	cloud.google.com/go v0.84.0
	contrib.go.opencensus.io/exporter/prometheus v0.2.0
	contrib.go.opencensus.io/exporter/stackdriver v0.13.8
	github.com/Azure/go-autorest v11.1.1+incompatible
	github.com/BurntSushi/toml v0.3.1
	github.com/Masterminds/goutils v1.1.0
	github.com/Masterminds/semver v1.4.2
	github.com/Masterminds/sprig v2.22.0+incompatible
	github.com/PuerkitoBio/purell v1.1.1
	github.com/PuerkitoBio/urlesc v0.0.0-20170810143723-de5bf2ad4578
	github.com/alessio/shellescape v0.0.0-20190409004728-b115ca0f9053
	github.com/beorn7/perks v1.0.1
	github.com/cenkalti/backoff v2.2.1+incompatible
	github.com/cyphar/filepath-securejoin v0.2.2
	github.com/davecgh/go-spew v1.1.1
	github.com/dgrijalva/jwt-go v3.2.0+incompatible
	github.com/felixge/httpsnoop v1.0.2
	github.com/fsnotify/fsnotify v1.4.9
	github.com/getlantern/httptest v0.0.0-20161025015934-4b40f4c7e590
	github.com/getlantern/mockconn v0.0.0-20190403061815-a8ffa60494a6
	github.com/ghodss/yaml v1.0.0
	github.com/go-logr/logr v0.1.0
	github.com/go-logr/zapr v0.1.0
	github.com/go-openapi/jsonpointer v0.19.3
	github.com/go-openapi/jsonreference v0.19.3
	github.com/go-openapi/spec v0.19.3
	github.com/go-openapi/swag v0.19.5
	github.com/gobwas/glob v0.2.3
	github.com/gogo/protobuf v1.3.2
	github.com/golang/gddo v0.0.0-20210115222349-20d68f94ee1f
	github.com/golang/glog v0.0.0-20160126235308-23def4e6c14b
	github.com/golang/groupcache v0.0.0-20210331224755-41bb18bfe9da
	github.com/golang/mock v1.6.0
	github.com/golang/protobuf v1.5.2
	github.com/google/btree v1.0.0
	github.com/google/go-cmp v0.5.6
	github.com/google/gofuzz v1.0.0
	github.com/google/uuid v1.1.2
	github.com/googleapis/gax-go v2.0.0+incompatible
	github.com/googleapis/gnostic v0.3.1
	github.com/gophercloud/gophercloud v0.1.0
	github.com/gregjones/httpcache v0.0.0-20180305231024-9cad4c3443a7
	github.com/hashicorp/golang-lru v0.5.4
	github.com/huandu/xstrings v1.2.0
	github.com/imdario/mergo v0.3.9
	github.com/inconshreveable/mousetrap v1.0.0
	github.com/jhump/protoreflect v1.10.0
	github.com/json-iterator/go v1.1.11
	github.com/liggitt/tabwriter v0.0.0-20181228230101-89fcab3d43de
	github.com/mailru/easyjson v0.7.0
	github.com/mattn/go-isatty v0.0.12
	github.com/matttproud/golang_protobuf_extensions v1.0.1
	github.com/mitchellh/copystructure v1.0.0
	github.com/mitchellh/go-server-timing v1.0.2-0.20201108055052-feb680ab92c2
	github.com/mitchellh/reflectwalk v1.0.1
	github.com/modern-go/concurrent v0.0.0-20180306012644-bacd9c7ef1dd
	github.com/modern-go/reflect2 v1.0.1
	github.com/motemen/go-loghttp v0.0.0-20170804080138-974ac5ceac27
	github.com/motemen/go-nuts v0.0.0-20180315145558-42c35bdb11c2
	github.com/onsi/gomega v1.10.1
	github.com/pelletier/go-toml v1.9.3
	github.com/petar/GoLLRB v0.0.0-20130427215148-53be0d36a84c
	github.com/peterbourgon/diskv v2.0.1+incompatible
	github.com/pkg/errors v0.9.1
	github.com/prometheus/client_golang v1.11.0
	github.com/prometheus/client_model v0.2.0
	github.com/prometheus/common v0.26.0
	github.com/prometheus/procfs v0.7.3
	github.com/spf13/cobra v1.2.1
	github.com/spf13/pflag v1.0.5
	go.opencensus.io v0.23.0
	go.uber.org/atomic v1.7.0
	go.uber.org/multierr v1.6.0
	go.uber.org/zap v1.17.0
	golang.org/x/crypto v0.0.0-20200622213623-75b288015ac9
	golang.org/x/net v0.0.0-20210805182204-aaa1db679c0d
	golang.org/x/sync v0.0.0-20210220032951-036812b2e83c
	golang.org/x/sys v0.0.0-20210908233432-aa78b53d3365
	golang.org/x/text v0.3.6
	golang.org/x/time v0.0.0-20191024005414-555d28b269f0
	gomodules.xyz/jsonpatch v2.0.1+incompatible
	google.golang.org/api v0.48.0
	google.golang.org/appengine v1.6.7
	google.golang.org/grpc v1.40.0
	google.golang.org/protobuf v1.27.1
	gopkg.in/h2non/gock.v1 v1.1.2
	gopkg.in/inf.v0 v0.9.1
	gopkg.in/yaml.v2 v2.4.0
	gopkg.in/yaml.v3 v3.0.0-20210107192922-496545a6307b
	k8s.io/api v0.17.13
	k8s.io/apiextensions-apiserver v0.17.13
	k8s.io/apimachinery v0.17.14-rc.0
	k8s.io/cli-runtime v0.17.13
	k8s.io/client-go v0.17.13
	k8s.io/helm v2.13.0+incompatible
	k8s.io/klog v1.0.0
	k8s.io/kube-openapi v0.0.0-20200410145947-bcb3869e6f29
	k8s.io/utils v0.0.0-20200619165400-6e3d28b6ed19
	sigs.k8s.io/controller-runtime v0.5.11
	sigs.k8s.io/kind v0.7.0
	sigs.k8s.io/kustomize v2.0.3+incompatible
	sigs.k8s.io/yaml v1.1.0
)

require (
	github.com/aws/aws-sdk-go v1.40.50 // indirect
	github.com/census-instrumentation/opencensus-proto v0.3.0 // indirect
	github.com/cespare/xxhash/v2 v2.1.2 // indirect
	github.com/go-kit/log v0.1.0 // indirect
	github.com/go-logfmt/logfmt v0.5.1 // indirect
	github.com/googleapis/gax-go/v2 v2.0.5 // indirect
	github.com/h2non/parth v0.0.0-20190131123155-b4df798d6542 // indirect
	github.com/jmespath/go-jmespath v0.4.0 // indirect
	github.com/prometheus/statsd_exporter v0.20.3 // indirect
	github.com/ryanuber/go-license v0.0.0-20180405065157-c69f41c2c8d6 // indirect
	golang.org/x/lint v0.0.0-20210508222113-6edffad5e616 // indirect
	golang.org/x/mod v0.4.2 // indirect
	golang.org/x/oauth2 v0.0.0-20210819190943-2bc19b11175f // indirect
	golang.org/x/tools v0.1.2 // indirect
	golang.org/x/xerrors v0.0.0-20200804184101-5ec99f83aff1 // indirect
	google.golang.org/genproto v0.0.0-20210921142501-181ce0d877f6 // indirect
)

replace gopkg.in/fsnotify.v1 v1.4.9 => github.com/fsnotify/fsnotify v1.4.9

replace github.com/googlecloudrobotics/core => ../

replace github.com/googlecloudrobotics/core/src => ./
