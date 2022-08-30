# Token Vendor (golang, WIP)

**NOTE**: The golang version of the token vendor is work in progress. For original Java-based implemention please refer to [src/java/com/cloudrobotics/tokenvendor/README.md].

The token vendor provides authentication for requests from the robots to our cloud environment.
The robots identity is generated during setup via a public-private key pair.
The token vendor provides APIs for registering robots through their public key and OAuth2 workflows for authenticating the signed requests from robots to cloud resources, for example to write logs to GCP Logging. 
The token vendor itself is stateless and all data is stored in GCP.

The following workflows are covered by the token vendor:

* Register a robot by its public key and a unique device identifier. The public key is stored in a cloud backend (currently Cloud IoT Core device registry)
* Retrieve a robot's public key through the device identifier
* Generate an scoped and time-limited IAM access token for access to GCP resources
* Validate a given IAM access token 
