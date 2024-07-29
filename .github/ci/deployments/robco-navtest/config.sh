#!/usr/bin/env bash

# Enable google cloud robotics layer 2
APP_MANAGEMENT=true

GCP_PROJECT_ID=robco-navtest
GCP_REGION=europe-west1
GCP_ZONE=europe-west1-c
CLOUD_ROBOTICS_SHARED_OWNER_GROUP=cloud-robotics-cloud-owner-acl@twosync.google.com
TERRAFORM_GCS_BUCKET="robco-team-terraform-state"
TERRAFORM_GCS_PREFIX="state/${GCP_PROJECT_ID}"
CLOUD_ROBOTICS_CONTAINER_REGISTRY=gcr.io/robco-team
PRIVATE_DOCKER_PROJECTS=robco-team
CLOUD_ROBOTICS_CTX=gke_robco-navtest_europe-west1-c_cloud-robotics
