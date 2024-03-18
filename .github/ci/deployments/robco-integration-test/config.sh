#!/usr/bin/env bash

# Enable cloud robotics layer 2
APP_MANAGEMENT=true

GCP_PROJECT_ID=robco-integration-test
GCP_REGION=europe-west1
GCP_ZONE=europe-west1-c
CLOUD_ROBOTICS_SHARED_OWNER_GROUP=cloud-robotics-cloud-owner-acl@twosync.google.com
CLOUD_ROBOTICS_DEPLOY_ENVIRONMENT=GCP-testing
TERRAFORM_GCS_BUCKET="robco-team-terraform-state"
TERRAFORM_GCS_PREFIX="state/${GCP_PROJECT_ID}"
CLOUD_ROBOTICS_CONTAINER_REGISTRY=gcr.io/robco-team
PRIVATE_DOCKER_PROJECTS=robco-team
