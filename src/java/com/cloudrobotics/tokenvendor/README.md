# Token vendor

The token vendor supports a collection of OAuth-related workflows. For that the
TokenVendorModule.java registers 4 handlers.

The PublicKey{Publish,Read}Handlers abstract the access to a device registry (a
public key store). Currently the codebase supports GCP's CloudIot-Registry and
an in-memory store that is only used for testing.

## Robot registration

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
* Impl: PublicKeyPublishHandler.java

## Public key retrieval

To verify messages send by a robot one can fetch the public key from the 
keystore using this method.

* URL: /apis/core.token-vendor/v1/public-key.read
* Method: GET
* URL Params:
  * device-id: unique device name (by default robot-<robot-id>)
* Response: application/x-pem-file
* Impl: PublicKeyReadHandler.java

## OAuth2 access token requests by robots

Robots sign JWTs with their local private keys. These get verified against the
public keys from the keystore. If the key is present and enabled, the token
vendor will hand out an OAuth access token for robot-service@ service account.

* URL: /apis/core.token-vendor/v1/token.oauth2
* Method: POST
* Body: JWT query (TokenSource)
* Response: application/json
* Impl: TokenVendorHandler.java

## AuthN/Z verification

Browsers or robots can query endpoints like the ws-proxy with authorization
headers or a `?token=` query parameter. They are already authenticated, and the
token vendor just checks that IAM authorizes the request.

* URL: /apis/core.token-vendor/v1/token.verify
* Method: GET
* URL Params:
  * robots: boolean to indicate if robot-service account tookens are allowed
* Response: only http status code
* Impl: VerificationHandler.java

## Interactive AuthN & AuthZ

We use the token vendor together with oauth2-proxy as an authentication and
authorization helper for nginx. This is essentially a poor man's IAP, used
because the GCE Ingress controller does not support IAP annotations on the GCE
objects it creates. Ingresses can be protected by it with an auth-url
annotation:

```
nginx.ingress.kubernetes.io/auth-url: "http://oauth2-proxy.default.svc.cluster.local/apis/core.token-vendor/v1/token.verify"
nginx.ingress.kubernetes.io/auth-signin: "https://${CLOUD_ROBOTICS_DOMAIN}/oauth2/start?rd=$escaped_request_uri"
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
