#!/usr/bin/env python3
"""Validation script for generated Istio manifests."""

import sys
import yaml


def validate_manifest(dst_file):
  """Validates that the given file contains valid Kubernetes documents.

  Args:
    dst_file: Path to the destination YAML file to validate.
  """
  with open(dst_file, "r") as f:
    text = f.read()

  clean_text = "\n".join(
      line
      for line in text.splitlines()
      if not line.strip().startswith("{{") and not line.strip().startswith("}}")
  )
  docs = [d for d in yaml.safe_load_all(clean_text) if d]

  if not docs:
    print("Error: 0 Kubernetes documents generated!", file=sys.stderr)
    sys.exit(1)

  print(f"YAML Verification Passed: {len(docs)} valid Kubernetes documents.")


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("Usage: validate_istio_manifest.py <dst_file>", file=sys.stderr)
    sys.exit(1)
  validate_manifest(sys.argv[1])
