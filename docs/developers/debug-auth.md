# Debugging authentication problems

Useful tips for working with Authentication and Authorization systems.

## Run a sample request with various credentials

You can call Cloud APIs with curl to see whether authorization works.

### Your own credentials

```bash
PROJECT_NUMBER=201199916163
curl -v -H "Content-Type: application/json" \
        -H "Authorization: Bearer $(gcloud auth application-default print-access-token)" \
        "https://cloudroboticssensordata.googleapis.com/v1eap/projects/${PROJECT_NUMBER}/sensors"
```

### Service account JSON file

You can create a JSON file with the robot account's credentials on
the [Cloud console's credentials page](https://console.cloud.google.com/apis/credentials).

```bash
PROJECT_NUMBER=201199916163
JSON_CREDENTIALS=/tmp/my-project-b7364a68fa92.json
curl -v -H "Content-Type: application/json" \
        -H "Authorization: Bearer $(GOOGLE_APPLICATION_CREDENTIALS=${JSON_CREDENTIALS) gcloud auth application-default print-access-token)" \
        "https://cloudroboticssensordata.googleapis.com/v1eap/projects/${PROJECT_NUMBER}/sensors"
```

### Get an OAuth token from IAM

The token vendor doesn't have its own keys, but instead calls IAM's
generateAccessToken method. You can emulate its behavior by using the [API
Explorer](https://developers.google.com/apis-explorer/#search/iam%20credentials/iamcredentials/v1/iamcredentials.projects.serviceAccounts.generateAccessToken)
to call `iamcredentials.projects.serviceAccounts.generateAccessToken`. The name
parameter is
`projects/-/serviceAccounts/robot-service@my-project.iam.gserviceaccount.com`,
and the `scope` is `https://www.googleapis.com/auth/cloud-platform`.

You can pass the returned access token in an Authorization header as above.

## Check whether the client's request is well-formed and authenticated

The easiest way to verify that metadata server and the gRPC client library
are doing the right thing is to use a logging HTTP server as the gRPC server.
Instead of setting the gRPC host to the Cloud API server
(`cloudroboticssensordata.googleapis.com`), you set it to an
HTTPS-capable server under your control. You need HTTPS support because
otherwise the gRPC library will rightfully decline to send access tokens.

Luckily, your Cloud Robotics Core setup already runs an HTTPS server. Suppose
you're calling the gRPC service
`google.cloud.robotics.sensordata.v1eap.SensorDataService`. You can
hook a very simple Python HTTP server into your cloud nginx setup like
this:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: debug
spec:
  server.py: |
    import BaseHTTPServer
    import SocketServer

    class MyHandler(BaseHTTPServer.BaseHTTPRequestHandler):

      def do_GET(self):
        print "Got request for ", self.path, " with auth ", self.headers.get('Authorization')

      def do_POST(self):
        print "Got request for ", self.path, " with auth ", self.headers.get('Authorization')

    httpd = SocketServer.TCPServer(("", 8080), MyHandler)
    httpd.serve_forever()
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: debug
spec:
  selector:
    matchLabels:
      app: debug
  replicas: 1
  template:
    metadata:
      labels:
        app: debug
    spec:
      containers:
      - name: python
        image: python:2
        args: ["python", "/src/server.py"]
        volumeMounts:
        - name: src-volume
          mountPath: /src
      volumes:
      - name: src-volume
        configMap:
          name: debug
---
apiVersion: v1
kind: Service
metadata:
  labels:
    app: debug
  name: debug
spec:
  ports:
  - name: http
    port: 8082
    protocol: TCP
    targetPort: 8080
  selector:
    app: debug
  type: ClusterIP
---
apiVersion: networking.k8s.io/v1beta1
kind: Ingress
metadata:
  annotations:
    kubernetes.io/ingress.class: nginx
    nginx.ingress.kubernetes.io/backend-protocol: HTTP
  name: debug
spec:
  rules:
  - host: www.endpoints.my-project.cloud.goog
    http:
      paths:
      - backend:
          serviceName: debug
          servicePort: 8080
        path: /google.cloud.robotics.sensordata.v1eap.SensorDataService
```

This will log the Authorization header to the pod's stdout, so you can view it
with `kubectl logs`. Save the token to a file (don't paste it into the command
line because it will end up in your shell history).

## Checking tokens with the tokeninfo service

You can check the token's contents and sanity with Google's tokeninfo endpoint:

```shell
curl https://oauth2.googleapis.com/tokeninfo?access_token=$(cat /tmp/token.txt)
```
