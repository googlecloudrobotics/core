# Token Vendor

The token vendor provides authentication for requests from the robots to our cloud environment.
The robots identity is generated during setup via a public-private key pair.
The token vendor provides APIs for registering robots through their public key and OAuth2 workflows for authenticating the signed requests from robots to cloud resources, for example to write logs to GCP Logging.
The token vendor itself is stateless and all data is stored in GCP.

The following workflows are covered by the token vendor:

* Register a robot by its public key and a unique device identifier. The public key is stored in a cloud backend.
* Retrieve a robot's public key through the device identifier
* Generate an scoped and time-limited IAM access token for access to GCP resources
* Validate a given IAM access token

## Public Key Backends

The token vendor supports multiple backends for storage of public keys for registered devices.

### Kubernetes Configmaps

The Kubernetes backend uses configmaps to store and lookup public keys.
The configmaps are stored in a configured namespace with the device identifier as name.
The public key is stored under a key in the configmap.
Devices can be removed by deleting the configmap.

### In-Memory

Stores public keys in-memory for testing.

Example:

```
# Run with memory backend
bazel run //src/go/cmd/token-vendor -- -verbose --project testproject --accepted_audience test --key-store IN_MEMORY
# Store test key
curl --data-binary "@api/v1/testdata/rsa_cert.pem" -H "Content-type: application/x-pem-file" -D - http://127.0.0.1:9090/apis/core.token-vendor/v1/public-key.publish?device-id=robot-dev-testuser
# Retrieve key
curl -D - http://127.0.0.1:9090/apis/core.token-vendor/v1/public-key.read?device-id=robot-dev-testuser
```

## API

### /public-key.publish: Robot registration

New robots get registered by a human administrator (authorized by an access
token on the request). The method add the provided public key to the configured
key store. Write access to the public key registry needs to be restricted to
refuse eg. robots to register other robots.

* URL: /apis/core.token-vendor/v1/public-key.publish
* Method: POST
* URL Params:
  * device-id: unique device name (by default robot-<robot-id>)
* Body: application/x-pem-file
* Response: only http status code

### /public-key.configure: Customize robot registration

Configure optional properties of the on-prem robot registration. This call needs
to be authorized by an access token for a human administrator).

* URL: /apis/core.token-vendor/v1/public-key.configure
* Method: POST
* URL Params:
  * device-id: unique device name (by default robot-<robot-id>)
* Body: json {
  service-account: str, defaults to robot-service@<gcp-project>.iam.gserviceaccount.com"
  service-account-delegate: str, optional intermediate delegate
}
* Response: only http status code

### /public-key.read: Public key retrieval

To verify messages send by a robot one can fetch the public key from the
keystore using this method.

* URL: /apis/core.token-vendor/v1/public-key.read
* Method: GET
* URL Params:
  * device-id: unique device name (by default robot-<robot-id>)
* Response: application/x-pem-file

### /token.oauth2: OAuth2 access token requests by robots

Robots sign JWTs with their local private keys. These get verified against the
public keys from the keystore. If the key is present and enabled, the token
vendor will hand out an OAuth access token for requested service account.
The service account must be either the default one (robot-service@) or the
account configured during registration (see /public-key.configure).

* URL: /apis/core.token-vendor/v1/token.oauth2
* Method: POST
* URL Params:
  * service-account: for which service account to return the access token,
    (robot-service@<gcp-project>.iam.gserviceaccount.com" by default)
* Body: JWT query (TokenSource)
* Response: application/json

### /token.verify: AuthN/Z verification

Browsers or robots can query endpoints like the ws-proxy with authorization
headers or a `?token=` query parameter. They are already authenticated, and the
token vendor just checks that IAM authorizes the request.

* URL: /apis/core.token-vendor/v1/token.verify
* Method: GET
* URL Params:
  * robots: boolean to indicate if robot-service account tookens are allowed
* Response: only http status code

Results are backed by a cache with a 5 minute lifetime to ease the load on the
IAM backend.

### /jwt.verify: AuthZ verification

Robots sign JWTs with their local private keys. These get verified against the
public keys from the keystore. If the key is present and enabled, the token
vendor will return status code 200.
This endpoint allows 3rd parties to do a check against the token-vendor before
the client reached the token vendor to retrieve an OAuth token.
It only validates whether the robot is known to the token vendor, there is no
further authentication or authorization done with this endpoint.

* URL: /apis/core.token-vendor/v1/jwt.verify
* Method: GET
* Headers:
  * Authorization: JWT that allows authorization
* Response: only http status code

## Interactive AuthN & AuthZ (with oauth2-proxy)

We use the token vendor together with oauth2-proxy as an authentication and
authorization helper for nginx. This is essentially a poor man's IAP, used
because the GCE Ingress controller does not support IAP annotations on the GCE
objects it creates. Ingresses can be protected by it with an auth-url
annotation:

```
nginx.ingress.kubernetes.io/auth-url: "http://oauth2-proxy.default.svc.cluster.local/apis/core.token-vendor/v1/token.verify"
nginx.ingress.kubernetes.io/auth-signin: "https://{{ .Values.domain }}/oauth2/start?rd=$escaped_request_uri"
```

nginx is set up to use the oauth2-proxy as an authentication proxy, and the
oauth2-proxy has its upstream set to the token vendor.

The request for unauthenticated users flows like this:

 1. nginx-ingress receives the request.
 1. nginx-ingress queries the auth-url.
 1. oauth2-proxy sees that there's no cookie attached, redirects the user to the
    Google sign-in page.
 1. When the user is signed in, oauth2-proxy's callback page sets the encrypted
    auth cookie and redirects the user back to the original URL.
 1. nginx-ingress again queries the auth-url.
 1. oauth2-proxy sees the auth cookie, decrypts it and forwards the access token
    to the token vendor.
 1. The token vendor sees the access token and checks the authorization with
    IAM.
 1. When authorization is fine, token vendor returns 200.
 1. nginx-ingress forwards the request to the backend.

## curl API Example Workflow

First set the name of your project:

```bash
PROJECT=testproject
```

Publish a key for the device `robot-dev-testuser`:

```bash
curl -D - --max-time 3 --data-binary "@api/v1/testdata/rsa_cert.pem" -H "Authorization: Bearer $(gcloud auth print-access-token)" -H "Content-type: application/x-pem-file" https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/public-key.publish?device-id=robot-dev-testuser
```

Optionally set extra options for the device:

```bash
curl -D - --max-time 3 -d '{"service-account":"svc@${PROJECT}.iam.gserviceaccount.com"}' -H "Content-Type: application/json" -H "Authorization: Bearer $(gcloud auth print-access-token)" -H "Content-type: application/x-pem-file" https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/public-key.configure?device-id=robot-dev-testuser
```

Read the key again:

```bash
curl -D - --max-time 3 -H "Authorization: Bearer $(gcloud auth print-access-token)" https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/public-key.read?device-id=robot-dev-testuser
```

Verify if your local user account has access to the human and robot ACL:

```bash
curl -D - --max-time 3 -H "Authorization: Bearer $(gcloud auth print-access-token)" https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/token.verify
```

and

```bash
curl -D - --max-time 3 -H "Authorization: Bearer $(gcloud auth print-access-token)" https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/token.verify?robots=true
```

Request a cloud access token for the robot. First generate a valid JWT using the intstructions at [testdata/README.md](api/v1/testdata/README.md). Afterwards use it to request the cloud token:

```bash
JWT=$(cat api/v1/testdata/jwt.bin)
curl -D - --max-time 3 --data-binary "grant_type=urn:ietf:params:oauth:grant-type:jwt-bearer&assertion=${JWT}" https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/token.oauth2
```

You can capture the token in `$TOKEN` with:

```bash
TOKEN=$(curl -s --max-time 3 --data-binary "grant_type=urn:ietf:params:oauth:grant-typ
e:jwt-bearer&assertion=${JWT}" https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/token.oauth2 | jq -r .access_token)
```

Verify if the token has access to the robots ACL (it should respond 200):

```bash
curl -D - --max-time 3 -H "Authorization: Bearer ${TOKEN}" https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/token.verify?robots=true
```

Verify if the token does *not* have access to the human ACL (it should respond 403):

```bash
curl -D - --max-time 3 -H "Authorization: Bearer ${TOKEN}" https://www.endpoints.${PROJECT}.cloud.goog/apis/core.token-vendor/v1/token.verify
```
