package v1

import (
	"bytes"
	"context"
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"net/url"
	"os"
	"path"
	"strconv"
	"strings"
	"testing"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/k8s"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/tokensource"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/kubernetes/fake"
)

const (
	testPubKey    = "testdata/rsa_cert.pem"
	jwtBodyPrefix = "grant_type=urn:ietf:params:oauth:grant-type:jwt-bearer&assertion="
	saName        = "robot-service@testproject.iam.gserviceaccount.com"

	jwtCorrect  = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJ0ZXN0YXVkIiwiaXNzIjoicm9ib3QtZGV2LXRlc3R1c2VyIiwiZXhwIjoxOTEzMzczMDEwLCJzY29wZXMiOiJ0ZXN0c2NvcGVzIiwiY2xhaW1zIjoidGVzdGNsYWltcyJ9.WJP0shiqynW9ZrmV4k78W3_nn_YA86XLK58IJYyqUF-8LAG92MraNqVqD0t6i-s90VBL64hCXlsA7zP3WlsMHOEvXCyRkGffhbJNIlJqIVTVfGvyF-ZmuaAr352n5kmKTrfTRi7h9LWTcvDgSosN438J8Jy9BT1FE9P-BHfyBUegZ15DWFAiAhz0r_Fgj7hAMXUnRdZfj3_dE0Nhi5IGs3L-0XzU-dE150ZJvtGMdIjc_QCqYHV3wtSgETKDYQoonD08n6g5GqC8nNkqrWFMttafLdPaDAsr8KWtj1dD1w9sw1YJClEzF9JOc63WNPZf8CgdU2enFW-V-2vHbUaekg"
	jwtWrongSig = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJ0ZXN0YXVkIiwiaXNzIjoicm9ib3QtZGV2LXRlc3R1c2VyIiwiZXhwIjoxOTEzMzczMDEwLCJzY29wZXMiOiIuLi4iLCJjbGFpbXMiOiIuLi4ifQ.krAYHjkConzVudfXJUMiDNbVHF3RwkvOAhSCyTvOaJdlJ6sxh-TjPXo6W0yVT31qjLwhl1NYI-JlhcHX7TLiZbLCbGVXlQN2Nn4LvpbGdAH0KvSJkthqX7ld9tlVQGdlOUHCE5bBDG_9uBtpdOAv1zKUTquhyDM0qWVrQV1qUVOtwBCO6nt21l1eXgTwz50FVN33f1ZmhZfHW1u7Dq_XwBJmHFwN3aiD0NZohU7MpQiz-0u94Q9yZ588IjdZEUhSEUKrVtJjoPcxDhrXxoRMA8iP8_bMeOHteiAdYeBVBwFhu1d8pfcn6uoZROYD1xB1LWDTJx4GfQh6v3wtAwFu7Q"
	jwtWrongAud = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJhYmMiLCJpc3MiOiJyb2JvdC1kZXYtdGVzdHVzZXIiLCJleHAiOjE5MTMzNzMwMTAsInNjb3BlcyI6Ii4uLiIsImNsYWltcyI6Ii4uLiJ9.XIoSfJl7QE51XUt7XHvZTomuXAAjVKWhnBhCgZl91-dGO9aF_pVu9sc_kR-MODoZci9pUKaLfqLTbZkNgkwGvApXF4GZ1DBu0uG6ewbNzIA-2l67xztnGw_M5DrQpLnq31HT1hRlvB9cXOYj2qtVfQaOhZtSPeHviYXj1NiPzHIWdyZKGIYu-gofkAZACEKKDd8HBRv6bLOzgrJ9sxlsyIB_O-FzpgoGSH-bKj9QEbSazx1j7AdICq1pJ_ER9ovb0qcYqg1JPToeEB1L-GFGwZp2JAnVp2rbbwPfjQTVlGmmAu-NUA5SjbjrNSjwDnQZDBBhmx75uToptJsnC_xZAw"
	jwtExpired  = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJ0ZXN0YXVkIiwiaXNzIjoicm9ib3QtZGV2LXRlc3R1c2VyIiwiZXhwIjowLCJzY29wZXMiOiJ0ZXN0c2NvcGVzIiwiY2xhaW1zIjoidGVzdGNsYWltcyJ9.VgalRggp3RrwarTeNSZu-lYcjSOyH7S7g_6BIxV_RisRavbwr1liTUXEKA1fjx5zF_1I_dsPC64wXkK14lAmJFI4pMm8-oLXMgSyUOGAqicYGrRm-CeZ_xJmA37ZCKyyf7ijGCdaAqNbtnsSER3wHTIG7ccbpcEUvb57nCQnTBzlEAVqDFXh9D-7Md2SUmWXCvmWomkALnPPg1xeeWjQygQvmbvFOo37ZgD-GbuvEYr2ccc1otJVvGtpSdJmFc1fGOWy9ZkPMR9VZyrmapYlImZTX7yOfcP-TLcbKRQegt3JJKgvffRP1dhxZaB5fTwT5o5ZTTh7aLap-MUUoZN4bg"
)

type RoundTripFunc func(req *http.Request) *http.Response

func (f RoundTripFunc) RoundTrip(req *http.Request) (*http.Response, error) {
	return f(req), nil
}

func NewTestHTTPClient(fn RoundTripFunc) *http.Client {
	return &http.Client{Transport: fn}
}

type publicKeyConfigureHandlerK8sTest struct {
	desc           string
	configmaps     []*corev1.ConfigMap
	method         string
	deviceID       string
	body           string
	wantStatusCode int
}

func TestPublicKeyConfigureHandlerWithK8s(t *testing.T) {
	var cases = []publicKeyConfigureHandlerK8sTest{
		{
			desc: "happy case",
			configmaps: []*corev1.ConfigMap{
				{
					TypeMeta: metav1.TypeMeta{
						Kind:       "ConfigMap",
						APIVersion: "v1",
					},
					ObjectMeta: metav1.ObjectMeta{
						Name:      "testdevice",
						Namespace: "default",
					},
					Data: map[string]string{"pubKey": "testkey"},
				},
			},
			method:         http.MethodPost,
			deviceID:       "testdevice",
			body:           "{}",
			wantStatusCode: http.StatusOK,
		},
		{
			desc:           "wrong method, bad request",
			method:         http.MethodGet,
			deviceID:       "testdevice",
			wantStatusCode: http.StatusBadRequest,
		},
		{
			desc:           "missing device-id, bad request",
			method:         http.MethodPost,
			body:           "{}",
			wantStatusCode: http.StatusBadRequest,
		},
		{
			desc:           "wrong device-id, not found",
			method:         http.MethodPost,
			deviceID:       "testdevice",
			body:           "{}",
			wantStatusCode: http.StatusNotFound,
		},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			runPublicKeyConfigureHandlerWithK8sCase(t, &test)
		})
	}
}

func runPublicKeyConfigureHandlerWithK8sCase(t *testing.T, test *publicKeyConfigureHandlerK8sTest) {
	// Setup fake K8s environment
	cs := fake.NewSimpleClientset()
	if err := populateK8sEnv(cs, "default", test.configmaps); err != nil {
		t.Fatal(err)
	}
	kcl, err := k8s.NewK8sRepository(context.TODO(), cs, "default")
	if err != nil {
		t.Fatal(err)
	}
	// Setup app & API handler
	tv, err := app.NewTokenVendor(context.TODO(), kcl, nil, nil, "aud", saName)
	if err != nil {
		t.Fatal(err)
	}
	h := HandlerContext{tv: tv}
	// Make API call
	rr := httptest.NewRecorder()
	req := mustNewRequest(t, test.method, "/anything", strings.NewReader(test.body))
	q := req.URL.Query()
	q.Add("device-id", test.deviceID)
	req.URL.RawQuery = q.Encode()
	h.publicKeyConfigureHandler(rr, req)
	// check API response
	if rr.Code != test.wantStatusCode {
		t.Errorf("wrong status code, is %d, want %d", rr.Code, test.wantStatusCode)
	}
	if rr.Code != http.StatusOK {
		return
	}
}

type publicKeyReadHandlerK8sTest struct {
	desc           string
	configmaps     []*corev1.ConfigMap
	deviceID       string
	wantKey        string
	wantStatusCode int
}

func TestPublicKeyReadHandlerWithK8s(t *testing.T) {
	var cases = []publicKeyReadHandlerK8sTest{
		{
			"key_found",
			[]*corev1.ConfigMap{
				{
					TypeMeta: metav1.TypeMeta{
						Kind:       "ConfigMap",
						APIVersion: "v1",
					},
					ObjectMeta: metav1.ObjectMeta{
						Name:      "testdevice",
						Namespace: "default",
					},
					Data: map[string]string{"pubKey": "testkey"},
				},
			},
			"testdevice",
			"testkey",
			http.StatusOK,
		},
		// key not found is not an error and returns an empty response
		{
			"key_not_found",
			[]*corev1.ConfigMap{},
			"testdevice",
			"",
			http.StatusNotFound,
		},
		// for malformed configmaps we expect an error
		{
			"malformed_configmap",
			[]*corev1.ConfigMap{
				{
					TypeMeta: metav1.TypeMeta{
						Kind:       "ConfigMap",
						APIVersion: "v1",
					},
					ObjectMeta: metav1.ObjectMeta{
						Name:      "testdevice",
						Namespace: "default",
					},
					// missing Data field
				},
			},
			"testdevice",
			"",
			http.StatusInternalServerError,
		},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			runPublicKeyReadHandlerWithK8sCase(t, &test)
		})
	}
}

func populateK8sEnv(env kubernetes.Interface, ns string, maps []*corev1.ConfigMap) error {
	for _, m := range maps {
		if _, err := env.CoreV1().ConfigMaps(ns).Create(context.TODO(), m, metav1.CreateOptions{}); err != nil {
			return err
		}
	}
	return nil
}

func runPublicKeyReadHandlerWithK8sCase(t *testing.T, test *publicKeyReadHandlerK8sTest) {
	// Setup fake K8s environment
	cs := fake.NewSimpleClientset()
	if err := populateK8sEnv(cs, "default", test.configmaps); err != nil {
		t.Fatal(err)
	}
	kcl, err := k8s.NewK8sRepository(context.TODO(), cs, "default")
	if err != nil {
		t.Fatal(err)
	}
	// Setup app & API handler
	tv, err := app.NewTokenVendor(context.TODO(), kcl, nil, nil, "aud", saName)
	if err != nil {
		t.Fatal(err)
	}
	h := HandlerContext{tv: tv}
	// Make API call
	rr := httptest.NewRecorder()
	req := mustNewRequest(t, "GET", "/anything", nil)
	q := req.URL.Query()
	q.Add("device-id", test.deviceID)
	req.URL.RawQuery = q.Encode()
	h.publicKeyReadHandler(rr, req)
	// check API response
	if rr.Code != test.wantStatusCode {
		t.Errorf("wrong status code, is %d, want %d", rr.Code, test.wantStatusCode)
	}
	if rr.Code != http.StatusOK {
		return
	}
	body, err := io.ReadAll(rr.Body)
	if err != nil {
		t.Fatal(err)
	}
	gotKey := string(body)
	if gotKey != test.wantKey {
		t.Errorf("readHandler(..) = %v, want %v", gotKey, test.wantKey)
	}
}

func mustRespBodyFromFile(t *testing.T, file string) io.ReadCloser {
	isResponseBody, err := os.ReadFile(file)
	if err != nil {
		t.Fatal(err.Error())
	}
	return io.NopCloser(bytes.NewBuffer(isResponseBody))
}

func mustNewRequest(t *testing.T, method, url string, body io.Reader) *http.Request {
	r, err := http.NewRequest(method, url, body)
	if err != nil {
		t.Fatal(err.Error())
	}
	return r
}

func mustFileOpen(t *testing.T, name string) io.Reader {
	f, err := os.Open(name)
	if err != nil {
		t.Fatal(err.Error())
	}
	return f
}

func mustFileToString(t *testing.T, name string) string {
	fp := mustFileOpen(t, name)
	bytes, err := io.ReadAll(fp)
	if err != nil {
		t.Fatal(err.Error())
	}
	return string(bytes)
}

type publicKeyPublishHandlerK8sTest struct {
	desc       string
	configmaps []*corev1.ConfigMap
	deviceID   string
	// read key before publish
	wantKeyBefore string
	// publish and read key again
	body    io.Reader
	wantKey string
}

func TestPublicKeyPublishHandlerWithK8s(t *testing.T) {
	var cases = []publicKeyPublishHandlerK8sTest{
		{
			// happy path where no device is registered yet
			"register_new_device",
			[]*corev1.ConfigMap{},
			"testdevice",
			"",
			mustFileOpen(t, path.Join("testdata", "rsa_cert.pem")),
			mustFileToString(t, path.Join("testdata", "rsa_cert.pem")),
		},
		{
			// happy path where the device is already registered
			"update_device_key",
			[]*corev1.ConfigMap{
				{
					TypeMeta: metav1.TypeMeta{
						Kind:       "ConfigMap",
						APIVersion: "v1",
					},
					ObjectMeta: metav1.ObjectMeta{
						Name: "testdevice",
					},
					Data: map[string]string{"pubKey": "testkey"},
				},
			},
			"testdevice",
			"testkey",
			mustFileOpen(t, path.Join("testdata", "rsa_cert.pem")),
			mustFileToString(t, path.Join("testdata", "rsa_cert.pem")),
		},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			runPublicKeyPublishHandlerWithK8sCase(t, &test)
		})
	}
}

func runPublicKeyPublishHandlerWithK8sCase(t *testing.T, test *publicKeyPublishHandlerK8sTest) {
	// Setup fake K8s environment
	cs := fake.NewSimpleClientset()
	if err := populateK8sEnv(cs, "default", test.configmaps); err != nil {
		t.Fatal(err)
	}
	kcl, err := k8s.NewK8sRepository(context.TODO(), cs, "default")
	if err != nil {
		t.Fatal(err)
	}
	// Setup app & API handler
	tv, err := app.NewTokenVendor(context.TODO(), kcl, nil, nil, "aud", saName)
	if err != nil {
		t.Fatal(err)
	}
	h := HandlerContext{tv: tv}
	// Read the current device key
	rr := httptest.NewRecorder()
	req := mustNewRequest(t, "GET", "/anything", nil)
	q := req.URL.Query()
	q.Add("device-id", test.deviceID)
	req.URL.RawQuery = q.Encode()
	h.publicKeyReadHandler(rr, req)
	if rr.Code != http.StatusOK && rr.Code != http.StatusNotFound {
		t.Errorf("before update,publicKeyReadHandler(..): wrong status code, got %d, want %d/%d",
			rr.Code, http.StatusOK, http.StatusNotFound)
	}
	body, err := io.ReadAll(rr.Body)
	if err != nil {
		t.Fatal(err)
	}
	if rr.Code == http.StatusOK {
		gotKey := string(body)
		if gotKey != test.wantKeyBefore {
			t.Errorf("before update,publicKeyReadHandler(..): wrong key, got %v, want %v",
				gotKey, test.wantKey)
		}
	}
	// POST a new key
	rr = httptest.NewRecorder()
	req = mustNewRequest(t, "POST", "/anything", test.body)
	q = req.URL.Query()
	q.Add("device-id", test.deviceID)
	req.URL.RawQuery = q.Encode()
	h.publicKeyPublishHandler(rr, req)
	// check API response
	if rr.Code != http.StatusOK {
		t.Errorf("publicKeyPublishHandler(..): wrong status code %d, want %d", rr.Code, http.StatusOK)
	}
	// Read key back again
	rr = httptest.NewRecorder()
	req = mustNewRequest(t, "GET", "/anything", nil)
	q = req.URL.Query()
	q.Add("device-id", test.deviceID)
	req.URL.RawQuery = q.Encode()
	h.publicKeyReadHandler(rr, req)
	if rr.Code != http.StatusOK {
		t.Errorf("after update,publicKeyReadHandler(..): wrong status code, got %d, want %d",
			rr.Code, http.StatusOK)
	}
	body, err = io.ReadAll(rr.Body)
	if err != nil {
		t.Fatal(err)
	}
	gotKey := string(body)
	if gotKey != test.wantKey {
		t.Errorf("after update,publicKeyReadHandler(..): wrong key, got %v, want %v",
			gotKey, test.wantKey)
	}
}

type isValidPublicKeyTest struct {
	desc    string
	pk      []byte
	isValid bool
}

func TestIsValidPublicKey(t *testing.T) {
	var cases = []isValidPublicKeyTest{
		{"testdata key", []byte(mustFileToString(t, testPubKey)), true},
		{"junk string", []byte("some junk string"), false},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			gotValid, err := isValidPublicKey(test.pk)
			if test.isValid != gotValid {
				t.Errorf("isValidPublicKey(..): is %v, got %v", test.isValid, gotValid)
			}
			if test.isValid && err != nil || !test.isValid && err == nil {
				t.Errorf("isValidPublicKey(..): is %v, but got error %v", test.isValid, err)
			}
		})
	}
}

type isValidTokenTest struct {
	desc    string
	token   string
	isValid bool
}

func TestIsValidToken(t *testing.T) {
	const p = "ya29."
	var cases = []isValidTokenTest{
		// valid
		{"valid", p + "a_bc-D0348" + strings.Repeat("a", 100), true},
		//invalid
		{"empty", "", false}, {"too short", "abc", false},
		{"wrong prefix", "ya244" + strings.Repeat("a", 100), false},
		{"new line", p + strings.Repeat("a", 100) + "\n" + p + strings.Repeat("a", 100), false},
		{"wrong characters", p + strings.Repeat("a", 100) + "#!", false},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			gotValid, gotErr := isValidToken(test.token)
			if gotValid != test.isValid {
				t.Errorf("isValidToken(%q): is %v, got %v", test.token, test.isValid, gotValid)
				return
			}
			if (test.isValid && gotErr != nil) || (!test.isValid && gotErr == nil) {
				t.Errorf("isValidToken(%q): is %v, but got error %v",
					test.token, test.isValid, gotErr)
			}
		})
	}
}

type isValidJWTTest struct {
	desc      string
	jwt       string
	wantValid bool
}

func TestIsValidJWT(t *testing.T) {
	var cases = []isValidJWTTest{
		// valid
		{"valid", strings.Repeat("a", 100) + "." + strings.Repeat("a", 100) +
			"." + strings.Repeat("a", 100), true},
		//invalid
		{"empty", "", false}, {"too short", "abc", false}, {"no dots", strings.Repeat("a", 100), false},
		{"new line", strings.Repeat("a", 100) + "\n" + strings.Repeat("a", 100), false},
		{"wrong characters", strings.Repeat("a", 100) + "#!", false},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			gotValid, gotErr := isValidJWT(test.jwt)
			if gotValid != test.wantValid {
				t.Errorf("isValidJWT(%q): got %v, want %v, error %v",
					test.jwt, gotValid, test.wantValid, gotErr)
				return
			}
			if (test.wantValid && gotErr != nil) || (!test.wantValid && gotErr == nil) {
				t.Errorf("isValidJWT(%q): got error %v, want %v",
					test.jwt, gotErr, test.wantValid)
			}
		})
	}
}

type tokenFromRequestTest struct {
	desc    string
	h       http.Header
	u       *url.URL
	isToken string
	isErr   bool
}

func TestTokenFromRequest(t *testing.T) {
	validToken := `ya29.a_bc-d` + strings.Repeat("a", 100)
	validUrl, _ := url.Parse("http://127.0.0.1:80/?token=" + validToken)
	var cases = []tokenFromRequestTest{
		{
			"token in auth header",
			http.Header{"Authorization": {"Bearer " + validToken}},
			&url.URL{},
			validToken,
			false,
		},
		{
			"invalid auth header",
			http.Header{"Authorization": {"SomethingElse: " + validToken}},
			&url.URL{},
			"",
			true,
		},
		{
			"token in x forwarded header",
			http.Header{"X-Forwarded-Access-Token": {validToken}},
			&url.URL{},
			validToken,
			false,
		},
		{
			"invalid token in x forwarded header",
			http.Header{"X-Forwarded-Access-Token": {"InvalidTokenString"}},
			&url.URL{},
			validToken,
			true,
		},
		{
			"token in URL",
			http.Header{},
			validUrl,
			validToken,
			false,
		},
		{
			"no token",
			http.Header{},
			&url.URL{},
			"",
			true,
		},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			gotToken, gotErr := tokenFromRequest(test.u, &test.h)
			if test.isErr && gotErr == nil || !test.isErr && gotErr != nil {
				t.Errorf("tokenFromRequest(..): error is %v, but got error %v", test.isErr, gotErr)
				return
			}
			if gotErr != nil {
				return
			}
			if gotToken != test.isToken {
				t.Errorf("tokenFromRequest(..): is %q, got %q", test.isToken, gotToken)
				return
			}
		})
	}
}

type VerifyTokenHandlerTest struct {
	desc string
	// handler request variables (test -> handler)
	reqIsRobots bool
	// IAM request variables (token verifier -> GCP IAM)
	iamReqIsUrl string
	// fake IAM response variables (GCP IAM -> token verifier)
	iamRespPermissions string
	iamRespStatusCode  int
	// handler response variables (handler -> test)
	handlerIsStatusCode int
}

func TestVerifyTokenHandler(t *testing.T) {
	const permHappy = `{"permissions":["iam.serviceAccounts.actAs"]}`
	const permBad = `{"permissions":[]}`
	const isUrlRobotsACL = "https://iam.googleapis.com/v1/projects/testproject/serviceAccounts/robot-service@testproject.iam.gserviceaccount.com:testIamPermissions?alt=json&prettyPrint=false"
	const isUrlHumanACL = "https://iam.googleapis.com/v1/projects/testproject/serviceAccounts/human-acl@testproject.iam.gserviceaccount.com:testIamPermissions?alt=json&prettyPrint=false"

	var cases = []VerifyTokenHandlerTest{
		{"human-acl happy path", false, isUrlHumanACL, permHappy, http.StatusOK, http.StatusOK},
		{"human-acl missing permission", false, isUrlHumanACL, permBad, http.StatusOK, http.StatusForbidden},
		{"robots-service happy path", true, isUrlRobotsACL, permHappy, http.StatusOK, http.StatusOK},
		{"error from GCP IAM", true, isUrlRobotsACL, permHappy, http.StatusBadRequest, http.StatusForbidden},
	}

	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			runVerifyTokenHandlerTest(t, &test)
		})
	}
}

func runVerifyTokenHandlerTest(t *testing.T, test *VerifyTokenHandlerTest) {
	isToken := "ya29." + strings.Repeat("a", 100)
	// fake IAM response
	fakeIAMHandler := func(req *http.Request) *http.Response {
		if req.Method != http.MethodPost {
			t.Fatalf("unexpected request method %q", req.Method)
		}
		if req.URL.String() != test.iamReqIsUrl {
			t.Fatalf("wrong POST URL, is %q, got %q", test.iamReqIsUrl, req.URL)
		}
		auth := req.Header.Get("Authorization")
		if !strings.HasPrefix(auth, "Bearer ") {
			t.Fatal("missing auth bearer prefix")
		}
		gotToken := strings.TrimPrefix(auth, "Bearer ")
		if isToken != gotToken {
			t.Fatalf("wrong token, is %q, got %q", isToken, gotToken)
		}
		// only check if the permission is in the request body and not unmarshal the whole json
		body, err := io.ReadAll(req.Body)
		if err != nil {
			t.Fatal(err.Error())
		}
		if !strings.Contains(string(body), "iam.serviceAccounts.actAs") {
			t.Fatalf("request does not contain expected permission")
		}
		// respond with the requested permissions
		return &http.Response{
			StatusCode: test.iamRespStatusCode,
			Body:       io.NopCloser(strings.NewReader(test.iamRespPermissions)),
			Header:     make(http.Header),
		}
	}

	// setup app and http client
	client := NewTestHTTPClient(fakeIAMHandler)
	tver, err := oauth.NewTokenVerifier(context.TODO(), client, "testproject")
	if err != nil {
		t.Fatal(err.Error())
	}
	tv, err := app.NewTokenVendor(context.TODO(), nil, tver, nil, "aud", saName)
	if err != nil {
		t.Fatal(err.Error())
	}
	h := &HandlerContext{tv: tv}

	// make request to the handler
	rr := httptest.NewRecorder()
	req := mustNewRequest(t, "GET", "/anything", nil)
	req.Header.Add("Authorization", "Bearer "+isToken)
	q := req.URL.Query()
	q.Add("robots", strconv.FormatBool(test.reqIsRobots))
	req.URL.RawQuery = q.Encode()
	h.verifyTokenHandler(rr, req)

	// check response
	if rr.Code != test.handlerIsStatusCode {
		t.Errorf("wrong status code, is %d, got %d", test.handlerIsStatusCode, rr.Code)
	}
}

type TokenOAuth2HandlerTest struct {
	desc string
	// token vendor config
	acceptedAud string
	scopes      []string
	// test -> handler
	body string
	// fake GCP -> Token Vendor
	token  string
	expire string
	// handler -> test
	wantStatusCode int
}

// we create a happy test first and afterwards variations of it
var TokenOAuth2HandlerTestHappyPath = TokenOAuth2HandlerTest{
	desc:        "happy path",
	acceptedAud: "testaud",
	scopes:      []string{"abc", "def"},
	// token defined in oauth/jwt/jwt_test.go
	body:  jwtBodyPrefix + jwtCorrect,
	token: "abc",
	// expire needs to be the same across all tests because checked the same across all tests
	expire:         "2100-06-30T15:01:23.045123456Z",
	wantStatusCode: 200,
}

func TestTokenOAuth2HandlerHapyPath(t *testing.T) {
	t.Run("with_k8s", func(t *testing.T) {
		runTokenOAuth2HandlerTestWithK8s(t, TokenOAuth2HandlerTestHappyPath)
	})
}

func TestTokenOAuth2HandlerDifferentPrivateKey(t *testing.T) {
	test := TokenOAuth2HandlerTestHappyPath
	test.desc = "JWT signed with different private key"
	// JWT is valid but signed with a different (random) private key not matching
	// the one returned from the registry for the given device
	test.body = jwtBodyPrefix + jwtWrongSig
	test.wantStatusCode = 403
	t.Run("with_k8s", func(t *testing.T) {
		runTokenOAuth2HandlerTestWithK8s(t, test)
	})
}

func TestTokenOAuth2HandlerWrongAud(t *testing.T) {
	test := TokenOAuth2HandlerTestHappyPath
	test.desc = "invalid JWT (junk audience)"
	// JWT "aud" is changed to "abc"
	test.body = jwtBodyPrefix + jwtWrongAud
	test.wantStatusCode = 403
	t.Run("with_k8s", func(t *testing.T) {
		runTokenOAuth2HandlerTestWithK8s(t, test)
	})
}

func TestTokenOAuth2HandlerExpired(t *testing.T) {
	test := TokenOAuth2HandlerTestHappyPath
	test.desc = "JWT signed correctly but expired"
	// JWT is valid but signed with a different (random) private key not matching
	// the one returned from the registry for the given device
	test.body = jwtBodyPrefix + jwtExpired
	test.wantStatusCode = 403
	t.Run("with_k8s", func(t *testing.T) {
		runTokenOAuth2HandlerTestWithK8s(t, test)
	})
}

func runTokenOAuth2HandlerTestWithK8s(t *testing.T, test TokenOAuth2HandlerTest) {
	// fake GCP IAM response for an access token
	fakeIAMAPI := func(req *http.Request) *http.Response {
		const wantUrl = "https://iamcredentials.googleapis.com/v1/projects/-/serviceAccounts/robot-service@testproject.iam.gserviceaccount.com:generateAccessToken?alt=json&prettyPrint=false"
		if req.URL.String() != wantUrl {
			t.Fatalf("wrong IAM URL, got %q, want %q", req.URL, wantUrl)
		}
		body := `{
			"accessToken": "` + test.token + `",
			"expireTime": "` + test.expire + `"
		  }`
		return &http.Response{
			StatusCode: http.StatusOK,
			Body:       io.NopCloser(strings.NewReader(body)),
			Header:     make(http.Header),
		}
	}
	// setup app and http client
	clientIAM := NewTestHTTPClient(fakeIAMAPI)
	ts, err := tokensource.NewGCPTokenSource(context.TODO(), clientIAM, test.scopes)
	if err != nil {
		t.Fatal(err)
	}
	cs := fake.NewSimpleClientset()
	if err = populateK8sEnv(cs, "default",
		[]*corev1.ConfigMap{
			{
				TypeMeta: metav1.TypeMeta{
					Kind:       "ConfigMap",
					APIVersion: "v1",
				},
				ObjectMeta: metav1.ObjectMeta{
					Name: "robot-dev-testuser",
				},
				Data: map[string]string{"pubKey": "-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAvTGUksynbWhvZkHNJn8C2oXVD400jiK4T0JoyS/SwbBGwFr3OJGlPwXCsvAPAzmpTuZpge6T3pnIcO/s97sMgyld9ZYio7SQiiRV/nwYZittGf9/yfHSNDJUvT25yhuK2p3UqRCom1a3KljeXbxXvGuYG48IH0kqAQbYBI/0lAV3H5pkdXPFZC6PHltC3jySVIOg7qPXrNuxdxmg/gmzQ9+NmKvXWKATAPax1yYoESaZtc22aCZWouIdJr3baYlfBb4w8stoJPoONuyn4ard17gywb46HHGl2XoY+Y5pihwvctsFeZXLfYwUmFPfgncQHJ02lCV3+Xyk4AAZy3xDpwIDAQAB\n-----END PUBLIC KEY-----"},
			},
		}); err != nil {
		t.Fatal(err)
	}
	r, err := k8s.NewK8sRepository(context.TODO(), cs, "default")
	if err != nil {
		t.Fatal(err)
	}
	tv, err := app.NewTokenVendor(context.TODO(), r, nil, ts, test.acceptedAud, saName)
	if err != nil {
		t.Fatal(err)
	}
	h := &HandlerContext{tv: tv}

	// make request to the handler
	rr := httptest.NewRecorder()
	req := mustNewRequest(t, "POST", "/anything", io.NopCloser(strings.NewReader(test.body)))
	h.tokenOAuth2Handler(rr, req)

	// check response
	if rr.Code != test.wantStatusCode {
		t.Fatalf("wrong status code, got %d, want %d", rr.Code, test.wantStatusCode)
	}
	// no body is provided in case of bad requests
	if test.wantStatusCode != 200 {
		return
	}
	var resp tokensource.TokenResponse
	err = json.Unmarshal(rr.Body.Bytes(), &resp)
	if err != nil {
		t.Fatalf("failed to unmarshal response body, got body %q", rr.Body)
	}
	if resp.AccessToken != test.token {
		t.Fatalf("Token(..) access token: got %q, want %q", resp.AccessToken, test.token)
	}
	wantScopes := strings.Join(test.scopes, " ")
	if resp.Scope != wantScopes {
		t.Fatalf("Token(..) scopes: got %v, want %v", resp.Scope, wantScopes)
	}
	if resp.TokenType != "Bearer" {
		t.Fatalf("Token(..) token type: got %q, want %q", resp.TokenType, "Bearer")
	}
	// test will fail in the year 2070
	if resp.ExpiresIn < 1_507_248_000 || resp.ExpiresIn > 2_453_852_873 {
		t.Fatalf("Token(..) expires in wrong, got %d, wanted far in the future, but not too far",
			resp.ExpiresIn)
	}
}

func Test_verifyJWTHandler(t *testing.T) {
	testCases := []struct {
		name         string
		headers      map[string][]string
		expectedCode int
	}{
		{
			name:         "success",
			headers:      map[string][]string{"Authorization": []string{jwtCorrect}},
			expectedCode: http.StatusOK,
		},
		{
			name:         "no-auth-header",
			expectedCode: http.StatusUnauthorized,
		},
		{
			name:         "wrong-sig",
			headers:      map[string][]string{"Authorization": []string{jwtWrongSig}},
			expectedCode: http.StatusForbidden,
		},
		{
			name:         "wrong-aud",
			headers:      map[string][]string{"Authorization": []string{jwtWrongAud}},
			expectedCode: http.StatusForbidden,
		},
		{
			name:         "expired",
			headers:      map[string][]string{"Authorization": []string{jwtExpired}},
			expectedCode: http.StatusForbidden,
		},
	}
	t.Parallel()

	cs := fake.NewSimpleClientset()
	if err := populateK8sEnv(cs, "default",
		[]*corev1.ConfigMap{
			{
				TypeMeta: metav1.TypeMeta{
					Kind:       "ConfigMap",
					APIVersion: "v1",
				},
				ObjectMeta: metav1.ObjectMeta{
					Name: "robot-dev-testuser",
				},
				Data: map[string]string{"pubKey": "-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAvTGUksynbWhvZkHNJn8C2oXVD400jiK4T0JoyS/SwbBGwFr3OJGlPwXCsvAPAzmpTuZpge6T3pnIcO/s97sMgyld9ZYio7SQiiRV/nwYZittGf9/yfHSNDJUvT25yhuK2p3UqRCom1a3KljeXbxXvGuYG48IH0kqAQbYBI/0lAV3H5pkdXPFZC6PHltC3jySVIOg7qPXrNuxdxmg/gmzQ9+NmKvXWKATAPax1yYoESaZtc22aCZWouIdJr3baYlfBb4w8stoJPoONuyn4ard17gywb46HHGl2XoY+Y5pihwvctsFeZXLfYwUmFPfgncQHJ02lCV3+Xyk4AAZy3xDpwIDAQAB\n-----END PUBLIC KEY-----"},
			},
		}); err != nil {
		t.Fatal(err)
	}
	r, err := k8s.NewK8sRepository(context.TODO(), cs, "default")
	if err != nil {
		t.Fatal(err)
	}
	tv, err := app.NewTokenVendor(context.TODO(), r, nil, nil, "testaud", saName)
	if err != nil {
		t.Fatal(err)
	}
	h := &HandlerContext{tv: tv}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()

			rr := httptest.NewRecorder()
			req := mustNewRequest(t, http.MethodGet, "", nil)
			req.Header = tc.headers

			h.verifyJWTHandler(rr, req)

			if rr.Result().StatusCode != tc.expectedCode {
				t.Errorf("verifyJWTHandler wrong status. Expected %v got %v", tc.expectedCode, rr.Result().StatusCode)
			}
		})
	}
}
