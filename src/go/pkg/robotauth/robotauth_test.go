package robotauth

import (
	"context"
	"crypto/rsa"
	"crypto/x509"
	"encoding/pem"
	"net/http"
	"net/http/httptest"
	"reflect"
	"testing"
	"testing/quick"
	"time"

	"golang.org/x/oauth2/jws"
	"k8s.io/client-go/kubernetes/fake"
)

// Test keys copied from token-vendor oauth/jwt
const testPubKey = `
-----BEGIN PUBLIC KEY-----
MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAvTGUksynbWhvZkHNJn8C
2oXVD400jiK4T0JoyS/SwbBGwFr3OJGlPwXCsvAPAzmpTuZpge6T3pnIcO/s97sM
gyld9ZYio7SQiiRV/nwYZittGf9/yfHSNDJUvT25yhuK2p3UqRCom1a3KljeXbxX
vGuYG48IH0kqAQbYBI/0lAV3H5pkdXPFZC6PHltC3jySVIOg7qPXrNuxdxmg/gmz
Q9+NmKvXWKATAPax1yYoESaZtc22aCZWouIdJr3baYlfBb4w8stoJPoONuyn4ard
17gywb46HHGl2XoY+Y5pihwvctsFeZXLfYwUmFPfgncQHJ02lCV3+Xyk4AAZy3xD
pwIDAQAB
-----END PUBLIC KEY-----`

const testPrivKey = `
-----BEGIN RSA PRIVATE KEY-----
MIIEogIBAAKCAQEAvTGUksynbWhvZkHNJn8C2oXVD400jiK4T0JoyS/SwbBGwFr3
OJGlPwXCsvAPAzmpTuZpge6T3pnIcO/s97sMgyld9ZYio7SQiiRV/nwYZittGf9/
yfHSNDJUvT25yhuK2p3UqRCom1a3KljeXbxXvGuYG48IH0kqAQbYBI/0lAV3H5pk
dXPFZC6PHltC3jySVIOg7qPXrNuxdxmg/gmzQ9+NmKvXWKATAPax1yYoESaZtc22
aCZWouIdJr3baYlfBb4w8stoJPoONuyn4ard17gywb46HHGl2XoY+Y5pihwvctsF
eZXLfYwUmFPfgncQHJ02lCV3+Xyk4AAZy3xDpwIDAQABAoH/bKMLrT/W4/wT+6PN
KU3FVbWDompywyssqlZ31Q6g9pdCCTIyw0jemlG0ewtdk3yIu8WS0Aku36NudWtP
pvDBPo+CZILRYS9N0AUNXBPl7sUA4OzVdCBnk5FTF1daV7N5CA+ZDXuDVa91fduJ
1ElSF9+weCKph0170Rsc74G570Q1ypoee/gdhkwwK5aYfTs+Z6fpaEnHaPzcwYkF
4QTsCshtoGZslmgZt8Tm7sfDDFWD20fmr1s350Ne1I7VYRFiyGbQI+IB+4pc9LSX
8CHcHIzHidKYTSG6YwpDsNRN/BkQklhsuLnNacMFFddO0IHIS0GlLBJbCRkN3b/n
/XC5AoGBAPZIN3VCpSEAw6OsM1zL4CBcq2dOb5b87rAeUmSkmW415fuyUNJJBcaf
1pliCQNeg9RzRDuHOs6BTU9i+fLcbOwSapFzGxzqnv4xmkHbj1Xs52Z+97HvKKld
xlQ/TF72WGITZVwmQWxJ9Rgx+bi7OirzOtQYoNpFoF5vHgyGrUZ7AoGBAMSosXUk
uLMzrZjH4Oetp8tq9Udyk7Xkk7booU7I0iPb/Dvadsuc9WZI+LP4R3iWmtLcJOUr
WyfliCLvbWtF4aW2vo7hvffe19krg/H26WEuBTuQGCZv8B5o8xHSecb7jbrKt9g6
r8I5kr+2tAZKLC6mtFdJgfSXNO9tveBxe+XFAoGBAIwQljnCJVeXr6wuCygDavv8
uB6QpTYhsz3GgOVsFzZuwNVcnEp77SUBUnL5JlccMa1pwKx6RB+dufIkQDK22duI
vcLqy8iuRq4aV7iMvgAIM7I/E2/GrEFma50OQsjfIXTlwwedWifUB+gyw+sjz/kN
S6/EMfbxEjuixlwpW/JxAoGBAKG5dM44F6hPPFijL0J3XcD8QZ+zCuQPiKZnopgO
sDmLJF/4Za9Gccze/5/I8sWpXMNBBRptUDZ8HTtVmK8aNdm4cfdAj5/y46EVlxl6
Cyy+0tDLzAB4F4h6mEI0y66mmkRdh1jL0lQwUo1Ua7Gsd68Zqr8JlVSWsJKhtf+I
c/JdAoGAFCSDby7ByX0W23Su3R28+9lWRSmNG79kLRLzlXsCwXTUTFh/TjAaEKgK
vwi8dtCSMNnJLCUXGx5cjTndgjTl8Woah0wy9XNNeIUjI8JPxIwXmmjppPKdCBI4
0ZyqQjgPJvwfY7lxFjE10ypv99QDlEbnwngt6bvSkY+6+DQTUDw=
-----END RSA PRIVATE KEY-----
`

// Generate random RobotAuth values to ensure any secret stored in the k8s
// secret can be retrieved without modification to the value
func TestK8sSecretLoadStoreRoundtrip(t *testing.T) {
	// FWIW, this should require a better generator to get valid k8s
	// namespace names generated, and create the namespace before
	// we try to store the secret in it.
	// I can only assume the fake server is super lenient here.
	if err := quick.Check(func(a RobotAuth, ns string) bool {
		cs := fake.NewSimpleClientset()

		if err := a.StoreInK8sSecret(context.TODO(), cs, ns); err != nil {
			t.Errorf("Failed to store k8s secret: %v", err)
		}

		l, err := LoadFromK8sSecret(context.TODO(), cs, ns)
		if err != nil {
			t.Errorf("Failed to read k8s secret: %v", err)
		}

		return reflect.DeepEqual(&a, l)
	}, nil); err != nil {
		t.Errorf("Failed to check roundtrip of Store/Load K8sSecret: %v", err)
	}
}

func TestCreateJWT(t *testing.T) {
	a := RobotAuth{
		PrivateKey: []byte(testPrivKey),
	}

	jwtk, err := a.CreateJWT(context.TODO(), time.Minute*10)
	if err != nil {
		t.Errorf("Failed to create JWT: %v", err)
	}

	p, _ := pem.Decode([]byte(testPubKey))
	if p == nil {
		t.Fatalf("Failed to pem decode pubkey")
	}
	parsedKey, err := x509.ParsePKIXPublicKey(p.Bytes)
	if err != nil {
		t.Fatalf("Failed to decode public key: %v", err)
	}
	parsed, ok := parsedKey.(*rsa.PublicKey)
	if !ok {
		t.Fatalf("Failed to cast public key to rsa")
	}

	if err := jws.Verify(jwtk, parsed); err != nil {
		t.Errorf("Failed to validate created JWT: %v", err)
	}
}

type mockRoundTripper struct {
	response *http.Response
}

func (rt *mockRoundTripper) RoundTrip(req *http.Request) (*http.Response, error) {
	return rt.response, nil
}

func makeTokenResponse(token string) *http.Response {
	recorder := httptest.NewRecorder()
	recorder.Header().Add("Content-Type", "application/json")
	recorder.WriteString(token)
	return recorder.Result()
}

const (
	// unsigned token with known fields, check/generate with https://jwt.io and
	// a throwaway private key:
	//   ssh-keygen -t rsa -b 4096 -m PEM -f jwtRS256.key
	testToken = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJyb2JvdC1yb2JvdC1zaW0iLCJhdWQiOiJodHRwczovL3d3dy5lbmRwb2ludHMubXktdGVzdC1wcm9qZWN0LmNsb3VkLmdvb2cvYXBpcy9jb3JlLnRva2VuLXZlbmRvci92MS90b2tlbi5vYXV0aDIiLCJleHAiOjE3NDE2MTU3MjEsImlhdCI6MTc0MTYxNDgxMSwic3ViIjoicm9ib3Qtc2ltIiwicHJuIjoicm9ib3Qtc2ltIn0."

	textTokenExpiryUnix = 1741615721
)

func TestCreateJWTSource(t *testing.T) {
	mockRT := &mockRoundTripper{}
	ts := &robotJWTSource{
		client: http.Client{
			Transport: mockRT,
		},
	}

	mockRT.response = makeTokenResponse(testToken)
	token, err := ts.Token()
	if err != nil {
		t.Fatalf("ts.Token() failed unexpectedly: %v", err)
	}

	if want := "Bearer"; token.TokenType != want {
		t.Errorf("token.TokenType = %q, want %q", token.TokenType, want)
	}
	if want := time.Unix(textTokenExpiryUnix, 0); token.Expiry != want {
		t.Errorf("token.Expiry = %v, want %v", token.Expiry, want)
	}
}
