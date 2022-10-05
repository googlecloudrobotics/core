# Testdata

## RSA Keys

These files were generated according to [Cloud IoT key generation instructions](https://cloud.google.com/iot/docs/how-tos/credentials/keys):

```shell
openssl genrsa -out rsa_private.pem 2048
openssl rsa -in rsa_private.pem -pubout -out rsa_cert.pem
```

## Create a test JWT

Go to [jwt.io](https://jwt.io/). Use the following header:

```json
{
	"alg": "RS256",
	"typ": "JWT"
}
```

Use the following payload, replace `${PROJECT}` with your cloud project identifier and update the expire timestamp:

```json
{
	"aud": "https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/token.oauth2",
	"iss": "robot-dev-testuser",
	"exp": 1913373010,
	"scopes": "unused",
	"claims": "unused"
}
```

Use `rsa_cert.pem` and `rsa_private.pem` as keys.


