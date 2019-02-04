These files were generated according to [Cloud IoT key generation instructions]
(https://cloud.google.com/iot/docs/how-tos/credentials/keys):

```shell
openssl genrsa -out rsa_private.pem 2048
openssl rsa -in rsa_private.pem -pubout -out rsa_cert.pem
openssl ecparam -genkey -name prime256v1 -noout -out ec_private.pem
openssl ec -in ec_private.pem -pubout -out ec_public.pem
```
