package jwt

import (
	"testing"

	"github.com/google/go-cmp/cmp"
)

/*
# Valid token
Header:
{

	"alg": "RS256",
	"typ": "JWT"

}
Payload:
{

	"aud": "testaud",
	"iss": "robot-dev-testuser",
	"exp": 1913373010,
	"scopes": "testscopes",
	"claims": "testclaims"

}
Signed using `testPrivKey`
*/

var testPayload = payload{
	Aud:    "testaud",
	Iss:    "robot-dev-testuser",
	Exp:    1913373010,
	Scopes: "testscopes",
	Claims: "testclaims",
}

const testJWT = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJ0ZXN0YXVkIiwiaXNzIjoicm9ib3QtZGV2LXRlc3R1c2VyIiwiZXhwIjoxOTEzMzczMDEwLCJzY29wZXMiOiJ0ZXN0c2NvcGVzIiwiY2xhaW1zIjoidGVzdGNsYWltcyJ9.WJP0shiqynW9ZrmV4k78W3_nn_YA86XLK58IJYyqUF-8LAG92MraNqVqD0t6i-s90VBL64hCXlsA7zP3WlsMHOEvXCyRkGffhbJNIlJqIVTVfGvyF-ZmuaAr352n5kmKTrfTRi7h9LWTcvDgSosN438J8Jy9BT1FE9P-BHfyBUegZ15DWFAiAhz0r_Fgj7hAMXUnRdZfj3_dE0Nhi5IGs3L-0XzU-dE150ZJvtGMdIjc_QCqYHV3wtSgETKDYQoonD08n6g5GqC8nNkqrWFMttafLdPaDAsr8KWtj1dD1w9sw1YJClEzF9JOc63WNPZf8CgdU2enFW-V-2vHbUaekg"

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

/*
Unused in key right now, but here for reference:
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
*/

func TestVerifySignature(t *testing.T) {
	err := VerifySignature(testJWT, testPubKey)
	if err != nil {
		t.Fatalf("VerifySignature(..): valid JWT failed verifiy, got: %v", err)
	}
}

func TestPayloadUnsafe(t *testing.T) {
	p, err := PayloadUnsafe(testJWT)
	if err != nil {
		t.Fatalf("PayloadUnsafe(..): valid payload failed, got %v", err)
	}
	if diff := cmp.Diff(p, &testPayload); diff != "" {
		t.Fatalf("PayloadUnsafe(..): got %+v, want %+v, diff: %v", p, testPayload, diff)
	}
}

func TestPayloadUnsafeInvalidPayload(t *testing.T) {
	// insert junk into the payload part
	invalidJWT := testJWT[:80] + "Z" + testJWT[80:]
	p, err := PayloadUnsafe(invalidJWT)
	if err == nil { // no error
		t.Fatalf("PayloadUnsafe(..) should have errored for invalid input, got payload %+v", p)
	}
}

func TestVerifySignatureInvalidSig(t *testing.T) {
	// insert junk into the signature part
	invalidSig := testJWT[:230] + "Z" + testJWT[230:]
	err := VerifySignature(invalidSig, testPubKey)
	if err == nil { // no error
		t.Fatal("VerifySignature(..): should have errored with invalid signature, but did not")
	}
}

/*
# Invalid signature algorithm
The following token uses `HS256` algorithm with the secret `somesecret`.
*/

const testJWTInvalidSigAlg = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdWQiOiJ0ZXN0YXVkIiwiaXNzIjoicm9ib3QtZGV2LXRlc3R1c2VyIiwiZXhwIjoxOTEzMzczMDEwLCJzY29wZXMiOiJ0ZXN0c2NvcGVzIiwiY2xhaW1zIjoidGVzdGNsYWltcyJ9.dXmTXpf3gS12z-Jkkw3ZTttvCxymqh03iCRd77DZCjE"

func TestVerifySignatureInvalidSigAlg(t *testing.T) {
	// we use the public key as second parameter because we know it does parse the
	// key first before checking the signature algorithm
	err := VerifySignature(testJWTInvalidSigAlg, testPubKey)
	if err == nil { // no error
		t.Fatal("VerifySignature(..): should have errored with invalid signature alg, but did not")
	}
}
