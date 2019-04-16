# Terraform

These files can be used to create, update or delete your personal development
or shared projects.

To update your user project, use deploy.sh:

```shell
./deploy.sh update my-project
```

To update shared projects such as the robolab project, run:

```shell
./deploy.sh update the-robolab-project
```

Our Terraform setup is special in two ways:
 1. The deploy.sh script passes some variables from the config file in
    the named project as input to enable you to switch projects naturally.
    Terraform doesn't read the config directly, but rather through a
    `src/bootstrap/cloud/terraform/terraform.tfvars` file that is managed by
    deploy.sh.
 2. The Terraform state is stored on GCS, not in your client. Each project has
    its own state file in the GCS bucket. When initializing
    ("terraform init"), you need to set -backend-config to set the right prefix.
     deploy.sh also takes care of that.
