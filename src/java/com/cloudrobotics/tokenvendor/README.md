# Token vendor

The token vendor supports a collection of OAuth-related workflows.

## OAuth2 access token requests by robots

Robots sign JWTs with their local private keys, and have public keys stored in
Cloud IoT. The token vendor will hand out OAuth access tokens for robot-service@
service account.

## AuthN/Z verification

Browsers or robots can query endpoints like the ws-proxy with authorization
headers or a `?token=` query parameter. They are already authenticated, and the
token vendor just checks that IAM authorizes the request.

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
