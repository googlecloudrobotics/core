package v1

import (
	"bytes"
	"context"
	"io"
	"net/http"
	"net/http/httptest"
	"net/url"
	"os"
	"path"
	"strings"
	"testing"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/cloudiot"
)

const testDataPath = "testdata/cloudiot"
const testPubKey = "testdata/rsa_cert.pem"

type isValidDeviceIDTest struct {
	deviceId string
	ret      bool
}

func TestValidateDeviceId(t *testing.T) {

	var isValidDeviceIDTests = []isValidDeviceIDTest{
		// invalid
		{"", false}, {`\\\\\\\\\\\\\`, false}, {"t\nt", false}, {"1", false},
		{"robot-dev-\ndevice", false}, {"1test", false},
		{"AAAAAAAAAAAAAAAAAAAarobot-dev-device-\neuwest1-test-com", false},
		// valid
		{"robot-dev-device", true}, {"TEST.com", true},
	}

	for _, test := range isValidDeviceIDTests {
		v := isValidDeviceID(test.deviceId)
		if v != test.ret {
			t.Errorf("isValidDeviceID(%q), got %v, want %v", test.deviceId, v, test.ret)
		}
	}
}

type RoundTripFunc func(req *http.Request) *http.Response

func (f RoundTripFunc) RoundTrip(req *http.Request) (*http.Response, error) {
	return f(req), nil
}

func NewTestHTTPClient(fn RoundTripFunc) *http.Client {
	return &http.Client{Transport: fn}
}

type publicKeyReadHandlerTest struct {
	desc           string
	isCalledUrl    string
	isResponseBody io.ReadCloser
	isKey          string
}

func mustRespBodyFromFile(t *testing.T, file string) io.ReadCloser {
	isResponseBody, err := os.ReadFile(file)
	if err != nil {
		t.Fatal(err.Error())
	}
	return io.NopCloser(bytes.NewBuffer(isResponseBody))
}

// Test publicKeyReadHandler with IoT repository using test http.Client.
//
// Only the happy path of receiving a single key or no key is tested right now.
func TestPublicKeyReadHandlerWithIoT(t *testing.T) {
	var cases = []publicKeyReadHandlerTest{
		{
			"happy_path",
			"https://cloudiot.googleapis.com/v1/projects/testproject/locations/testregion/registries/testregistry/devices/testid?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false",
			mustRespBodyFromFile(t, path.Join(testDataPath, "describe_device.json")),
			"a_public_key",
		},
		{
			"happy_path_expired_key",
			"https://cloudiot.googleapis.com/v1/projects/testproject/locations/testregion/registries/testregistry/devices/testid?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false",
			mustRespBodyFromFile(t, path.Join(testDataPath, "describe_device_expired_key.json")),
			"",
		},
	}

	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			runPublicKeyReadHandlerWithIoTCase(t, &test)
		})
	}
}

func runPublicKeyReadHandlerWithIoTCase(t *testing.T, test *publicKeyReadHandlerTest) {
	fakeIoTHandler := func(req *http.Request) *http.Response {
		gotCalledUrl := req.URL.String()
		if gotCalledUrl != test.isCalledUrl {
			t.Errorf("request URL missmatch, is %q, got %q", gotCalledUrl, test.isCalledUrl)
		}
		return &http.Response{
			StatusCode: http.StatusOK,
			Body:       test.isResponseBody,
			Header:     make(http.Header),
		}
	}
	client := NewTestHTTPClient(fakeIoTHandler)
	reg := cloudiot.Registry{Project: "testproject", Region: "testregion", Registry: "testregistry"}
	r, err := cloudiot.NewCloudIoTRepository(context.TODO(), reg, client)
	if err != nil {
		t.Fatal(err.Error())
	}
	tv, err := app.NewTokenVendor(context.TODO(), r)
	if err != nil {
		t.Fatal(err.Error())
	}
	key, err := tv.ReadPublicKey(context.TODO(), "testid")
	if err != nil {
		t.Errorf(err.Error())
	}
	if key != test.isKey {
		t.Errorf("public key response does not match with the test data, is %q, got %q",
			test.isKey, key)
	}
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
		t.Fatalf(err.Error())
	}
	return string(bytes)
}

func mustSetupAppHandlerWithIoT(t *testing.T, client *http.Client) *HandlerContext {
	reg := cloudiot.Registry{Project: "testproject", Region: "testregion", Registry: "testregistry"}
	r, err := cloudiot.NewCloudIoTRepository(context.TODO(), reg, client)
	if err != nil {
		t.Fatal(err.Error())
	}
	tv, err := app.NewTokenVendor(context.TODO(), r)
	if err != nil {
		t.Fatal(err.Error())
	}
	return &HandlerContext{tv: tv}
}

func TestPublicKeyPublishHandlerWithIoT(t *testing.T) {
	t.Run("happy path with IoT", func(t *testing.T) {
		runPublicKeyPublishHandlerWithIoTHappyPath(t)
	})
}

func runPublicKeyPublishHandlerWithIoTHappyPath(t *testing.T) {
	// fake Cloud IoT responses
	// two calls are faked:
	// 1st call: get the device from the registry (GET)
	// 2st call: update the public key (PATCH)
	fakeIoTHandler := func(req *http.Request) *http.Response {
		// 1st call: GET is used by get device
		if req.Method == http.MethodGet {
			const isUrl = "https://cloudiot.googleapis.com/v1/projects/testproject/locations/testregion/registries/testregistry/devices/testdevice?alt=json&fieldMask=blocked&prettyPrint=false"
			if req.URL.String() != isUrl {
				t.Errorf("wrong get device URL, is %q, got %q", isUrl, req.URL)
			}
			return &http.Response{
				StatusCode: http.StatusOK,
				Body:       mustRespBodyFromFile(t, path.Join(testDataPath, "describe_device.json")),
				Header:     make(http.Header),
			}
		}
		// 2nd call: PATCH is used to update the public key
		if req.Method == http.MethodPatch {
			const isUrl = "https://cloudiot.googleapis.com/v1/projects/testproject/locations/testregion/registries/testregistry/devices/testdevice?alt=json&prettyPrint=false&updateMask=credentials"
			if req.URL.String() != isUrl {
				t.Errorf("wrong patch device URL, is %q, got %q", isUrl, req.URL)
			}
			// body of request should contain the public key of the device
			body, err := io.ReadAll(req.Body)
			if err != nil {
				t.Fatalf(err.Error())
			}
			// remove encoded and non-encoded new line characters for string match
			bodyStr := strings.ReplaceAll(string(body), "\\n", "")
			pem := mustFileToString(t, testPubKey)
			pem = strings.ReplaceAll(pem, "\n", "")
			// only check if it is in the request body and not unmarshal the whole json
			if !strings.Contains(bodyStr, pem) {
				t.Errorf("response does not contain expected public key")
			}
			return &http.Response{
				StatusCode: http.StatusOK,
				Body:       io.NopCloser(strings.NewReader("{}")),
				Header:     make(http.Header),
			}
		}
		t.Fatalf("unexpected request method %q", req.Method)
		return nil
	}
	client := NewTestHTTPClient(fakeIoTHandler)
	h := mustSetupAppHandlerWithIoT(t, client)

	rr := httptest.NewRecorder()
	req := mustNewRequest(t, "POST", "/anything", mustFileOpen(t, testPubKey))
	q := req.URL.Query()
	q.Add("device-id", "testdevice")
	req.URL.RawQuery = q.Encode()

	// call handler
	h.publicKeyPublishHandler(rr, req)

	// check response
	if rr.Code != http.StatusOK {
		t.Errorf("wrong status code, is %d, got %d", http.StatusOK, rr.Code)
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
			http.Header{"Authorization": {"Bearer: " + validToken}},
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
