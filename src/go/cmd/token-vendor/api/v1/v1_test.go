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
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/cloudiot"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/tokensource"
)

const testDataPath = "testdata/cloudiot"
const testPubKey = "testdata/rsa_cert.pem"

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
			mustFileToString(t, testPubKey),
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
	tv, err := app.NewTokenVendor(context.TODO(), r, nil, nil, "aud")
	if err != nil {
		t.Fatal(err.Error())
	}
	key, err := tv.ReadPublicKey(context.TODO(), "testid")
	if err != nil {
		t.Errorf(err.Error())
	}
	if strings.ReplaceAll(key, "\n", "") != strings.ReplaceAll(test.isKey, "\n", "") {
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
	tv, err := app.NewTokenVendor(context.TODO(), r, nil, nil, "aud")
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
	tv, err := app.NewTokenVendor(context.TODO(), nil, tver, nil, "aud")
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
	body:  "grant_type=urn:ietf:params:oauth:grant-type:jwt-bearer&assertion=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJ0ZXN0YXVkIiwiaXNzIjoicm9ib3QtZGV2LXRlc3R1c2VyIiwiZXhwIjoxOTEzMzczMDEwLCJzY29wZXMiOiJ0ZXN0c2NvcGVzIiwiY2xhaW1zIjoidGVzdGNsYWltcyJ9.WJP0shiqynW9ZrmV4k78W3_nn_YA86XLK58IJYyqUF-8LAG92MraNqVqD0t6i-s90VBL64hCXlsA7zP3WlsMHOEvXCyRkGffhbJNIlJqIVTVfGvyF-ZmuaAr352n5kmKTrfTRi7h9LWTcvDgSosN438J8Jy9BT1FE9P-BHfyBUegZ15DWFAiAhz0r_Fgj7hAMXUnRdZfj3_dE0Nhi5IGs3L-0XzU-dE150ZJvtGMdIjc_QCqYHV3wtSgETKDYQoonD08n6g5GqC8nNkqrWFMttafLdPaDAsr8KWtj1dD1w9sw1YJClEzF9JOc63WNPZf8CgdU2enFW-V-2vHbUaekg",
	token: "abc",
	// expire needs to be the same across all tests because checked the same across all tests
	expire:         "2100-06-30T15:01:23.045123456Z",
	wantStatusCode: 200,
}

func TestTokenOAuth2HandlerHapyPath(t *testing.T) {
	runTokenOAuth2HandlerTest(t, TokenOAuth2HandlerTestHappyPath)
}

func TestTokenOAuth2HandlerDifferentPrivateKey(t *testing.T) {
	test := TokenOAuth2HandlerTestHappyPath
	test.desc = "JWT signed with different private key"
	// JWT is valid but signed with a different (random) private key not matching
	// the one returned from the registry for the given device
	test.body = "grant_type=urn:ietf:params:oauth:grant-type:jwt-bearer&assertion=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJ0ZXN0YXVkIiwiaXNzIjoicm9ib3QtZGV2LXRlc3R1c2VyIiwiZXhwIjoxOTEzMzczMDEwLCJzY29wZXMiOiIuLi4iLCJjbGFpbXMiOiIuLi4ifQ.krAYHjkConzVudfXJUMiDNbVHF3RwkvOAhSCyTvOaJdlJ6sxh-TjPXo6W0yVT31qjLwhl1NYI-JlhcHX7TLiZbLCbGVXlQN2Nn4LvpbGdAH0KvSJkthqX7ld9tlVQGdlOUHCE5bBDG_9uBtpdOAv1zKUTquhyDM0qWVrQV1qUVOtwBCO6nt21l1eXgTwz50FVN33f1ZmhZfHW1u7Dq_XwBJmHFwN3aiD0NZohU7MpQiz-0u94Q9yZ588IjdZEUhSEUKrVtJjoPcxDhrXxoRMA8iP8_bMeOHteiAdYeBVBwFhu1d8pfcn6uoZROYD1xB1LWDTJx4GfQh6v3wtAwFu7Q"
	test.wantStatusCode = 403
	runTokenOAuth2HandlerTest(t, test)
}

func TestTokenOAuth2HandlerWrongAud(t *testing.T) {
	test := TokenOAuth2HandlerTestHappyPath
	test.desc = "invalid JWT (junk audience)"
	// JWT "aud" is changed to "abc"
	test.body = "grant_type=urn:ietf:params:oauth:grant-type:jwt-bearer&assertion=eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJhYmMiLCJpc3MiOiJyb2JvdC1kZXYtdGVzdHVzZXIiLCJleHAiOjE5MTMzNzMwMTAsInNjb3BlcyI6Ii4uLiIsImNsYWltcyI6Ii4uLiJ9.XIoSfJl7QE51XUt7XHvZTomuXAAjVKWhnBhCgZl91-dGO9aF_pVu9sc_kR-MODoZci9pUKaLfqLTbZkNgkwGvApXF4GZ1DBu0uG6ewbNzIA-2l67xztnGw_M5DrQpLnq31HT1hRlvB9cXOYj2qtVfQaOhZtSPeHviYXj1NiPzHIWdyZKGIYu-gofkAZACEKKDd8HBRv6bLOzgrJ9sxlsyIB_O-FzpgoGSH-bKj9QEbSazx1j7AdICq1pJ_ER9ovb0qcYqg1JPToeEB1L-GFGwZp2JAnVp2rbbwPfjQTVlGmmAu-NUA5SjbjrNSjwDnQZDBBhmx75uToptJsnC_xZAw"
	test.wantStatusCode = 403
	runTokenOAuth2HandlerTest(t, test)
}

func runTokenOAuth2HandlerTest(t *testing.T, test TokenOAuth2HandlerTest) {
	// fake Cloud IoT responses
	fakeIoTHandler := func(req *http.Request) *http.Response {
		const wantUrl = "https://cloudiot.googleapis.com/v1/projects/testproject/locations/testregion/registries/testregistry/devices/robot-dev-testuser?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false"
		if req.URL.String() != wantUrl {
			t.Fatalf("wrong get device URL, got %q, want %q", req.URL, wantUrl)
		}
		return &http.Response{
			StatusCode: http.StatusOK,
			Body:       mustRespBodyFromFile(t, path.Join(testDataPath, "describe_device.json")),
			Header:     make(http.Header),
		}
	}
	// fake GCP IAM response for an access token
	fakeIAMAPI := func(req *http.Request) *http.Response {
		const wantUrl = "https://iamcredentials.googleapis.com/v1/projects/-/serviceAccounts/testsa@testproject.iam.gserviceaccount.com:generateAccessToken?alt=json&prettyPrint=false"
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
	clientIoT := NewTestHTTPClient(fakeIoTHandler)
	rep, err := cloudiot.NewCloudIoTRepository(context.TODO(),
		cloudiot.Registry{Project: "testproject", Region: "testregion", Registry: "testregistry"},
		clientIoT)
	if err != nil {
		t.Fatal(err)
	}
	ts, err := tokensource.NewGCPTokenSource(context.TODO(), clientIAM, "testproject", "testsa",
		test.scopes)
	if err != nil {
		t.Fatal(err)
	}
	tv, err := app.NewTokenVendor(context.TODO(), rep, nil, ts, test.acceptedAud)
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
