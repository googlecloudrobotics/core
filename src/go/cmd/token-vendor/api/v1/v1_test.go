package v1

import (
	"bytes"
	"context"
	"io"
	"net/http"
	"net/http/httptest"
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
