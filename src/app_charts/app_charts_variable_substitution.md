# App Chart Variable Substitution Guide

This guide covers how to bypass Helm's static schema validation when configuring upstream charts via Bazel, using delayed Go `text/template` execution.

In other words, this document explains why there's so many ${CR_XXX} substitution pseudovariables, and the difference between `values-xxx.yaml` and `xxx.values.yaml` files.

## Motivation: Helm Schema Validation

When wrapping external Helm charts (e.g., `victoria-metrics-k8s-stack`), all configuration must pass through a `values.yaml` file. If you attempt to use Go template syntax `{{ .Values.x }}` as a value for an array or map, Helm wraps the template tag in double quotes during the build phase (`tolerations: "{{ .Values.x }}"`). This produces invalid Kubernetes YAML after deployment-time substitution.

**Solution**: Inject static bypass variables (e.g., `$CR_...: REMOVE_ME`) during the build phase to satisfy Helm, then use an `operator.yaml` script to substitute the actual runtime values directly into the raw YAML before applying to the cluster.

---

## Substitution Cheatsheet

### 1. Scalars (Strings, Integers, Booleans)

*   **Build-Time (`values.yaml`)**:

    ```yaml
    replicaCount: "${VMSTORAGE_REPLICA_COUNT}"
    ```

*   **Deploy-Time (`operator.yaml`)**:

    ```gotemplate
    {{- $data = $data | replace "${VMSTORAGE_REPLICA_COUNT}" (toString .Values.replicaCount) -}}
    ```

### 2. Maps (Dictionaries)

Requires a literal map with a bypass key to pass validation. The `nindent` must match the parent key's indentation.

*   **Build-Time (`values.yaml`)**:

    ```yaml
      nodeSelector: # 6 spaces indent
        $CR_VMSTORAGE_NODE_SELECTOR: REMOVE_ME
    ```

*   **Deploy-Time (`operator.yaml`)**:

    ```gotemplate
    {{- if .Values.nodeSelector }}
      {{- $nodeSelector := toYaml .Values.nodeSelector | nindent 6 -}}
      {{- $data = $data | replace "$CR_VMSTORAGE_NODE_SELECTOR: REMOVE_ME" (trimPrefix "      " (trimPrefix "\n" $nodeSelector)) -}}
    {{- else }}
      {{- $data = $data | replace "$CR_VMSTORAGE_NODE_SELECTOR: REMOVE_ME" "{}" -}}
    {{- end }}
    ```

> **Crucial**: You MUST include the `else` block. If the user provides
> no overrides, the bypass variable must be replaced with `{}` or it
> will cause a Kubernetes API schema error.

### 3. Lists (Arrays)

Requires a literal list with a bypass item. **Critical:** The `nindent` must match the exact indentation of the `-` character.

*   **Build-Time (`values.yaml`)**:

    ```yaml
      tolerations:
        - $CR_VMSTORAGE_TOLERATIONS: REMOVE_ME # '-' is at 8 spaces
    ```

*   **Deploy-Time (`operator.yaml`)**:

    ```gotemplate
    {{- if .Values.tolerations }}
      {{- $tolerations := toYaml .Values.tolerations | nindent 8 -}}
      {{- $data = $data | replace "- $CR_VMSTORAGE_TOLERATIONS: REMOVE_ME" (trimPrefix "        " (trimPrefix "\n" $tolerations)) -}}
    {{- else }}
      {{- $data = $data | replace "      tolerations:\n        - $CR_VMSTORAGE_TOLERATIONS: REMOVE_ME\n" "" -}}
    {{- end }}
    ```

> **Crucial**: You MUST include the `else` block to completely remove
> the parent key block (`tolerations:` and the bypass string) if the
> user provides no overrides.

#### The Indentation Math for Lists

If the list item prefix (`-`) is indented by 8 spaces, `nindent 8` outputs:
`\n        - key: value`
We use `trimPrefix` to strip the newline and the 8 spaces, yielding `- key: value`. When this replaces the bypass variable, the YAML list properties align perfectly.

### 4. Schema-Constrained Lists (Required Fields)

If an array's items have strictly enforced schemas in the upstream Helm chart
(e.g., `remoteWrite` requires a `url` field, or `containers` requires a `name`
field), a simple `- $CR_...: REMOVE_ME` string will fail Helm validation during
the build phase.

Instead, provide a dummy object that satisfies the schema, using the bypass
variable for the required field's value.

*   **Build-Time (`values.yaml`)**:

    ```yaml
      remoteWrite:
        - url: "${CR_REMOTE_WRITE_OBJECT}"
    ```

*   **Deploy-Time (`operator.yaml`)**:
    Target the specific dummy string representation for replacement (and removal
    if empty).

    ```gotemplate
    {{- if .Values.remoteWrite }}
      {{- $remoteWriteList := toYaml .Values.remoteWrite | nindent 2 -}}
      {{- $data = $data | replace "  - url: \"${CR_REMOTE_WRITE_OBJECT}\"" (printf "\n%s" (trimPrefix "\n" $remoteWriteList)) -}}
    {{- else }}
      {{- $data = $data | replace "  - url: \"${CR_REMOTE_WRITE_OBJECT}\"\n" "" -}}
    {{- end }}
    ```

### 5. Dynamic Evaluation in Deploy-Time Values (`tpl`)

If you want `values-cloud.yaml` to contain dynamic template substitutions (e.g., dynamically referring to `.Values.domain`), you **cannot** simply inject a Go template `{{ .Values.domain }}` via standard string replacement because `values-cloud.yaml` is the input parameter file, not the template itself.

Instead, you can pass the extracted value through Helm's `tpl` function inside `operator.yaml` before performing the string replacement.

*   **Deploy-Time Config (`values-cloud.yaml`)**:

    ```yaml
    prom_external_url: "https://{{ .Values.domain }}/prometheus/"
    ```

*   **Deploy-Time Script (`operator.yaml`)**:

    ```gotemplate
    {{- $externalUrl := tpl .Values.prom_external_url . -}}
    {{- $data = $data | replace "${EXTERNAL_URL}" $externalUrl -}}
    ```

This evaluates the string template found in `values-cloud.yaml` against the current `.Values` context before replacing the build-time placeholder.

---

## Architecture Context

The deployment flow uses a two-phase expansion:

1.   **Static Build Phase (Bazel/Helm 3)**: Bazel evaluates the upstream Helm charts using `*-cloud.values.yaml`. This generates a raw, intermediate YAML file containing the static bypass variables.
2.   **Deploy Phase (Synk/Operator)**: The `chartassignment-controller` receives user configurations via `values-cloud.yaml`. It executes the `*-operator.yaml` Go `text/template` scripts to format the user data and overwrite the bypass variables in the intermediate YAML.

*(Note: For directly authored templates like custom `rollout.yaml` files, bypass variables are unnecessary. Use standard `{{ .Values.x }}` substitution natively).*

---

## Accessing Intermediate Artifacts

If you need to inspect or debug the intermediate YAML generated after the static build phase (which still contains the unresolved `$CR_...` pseudo-variables before they are substituted out), you can extract it directly from the compiled App Chart package.

After running `bazel build` on an App Chart target, the `.tgz` package is written locally into `bazel-bin/`.

To view the intermediate artifact:
1. Locate the package: `bazel-bin/src/app_charts/<app_name>/<chart_name>-<version>.tgz`
2. Extract the tarball: The intermediate artifacts generated by Helm are located within the `files/` subdirectory of the extracted archive (e.g., `files/<chart_name>.yaml`).

---

## Local Testing Procedure

Verify template alignment locally to prevent YAML unmarshaling and syntax errors.

```bash
# 1. Build the chart artifact
bazel build //src/app_charts/victoriametrics/victoriametrics-cloudmetrics:victoriametrics-cloudmetrics-cloud

# 2. Extract the intermediate YAML (the output tarball is located in bazel-bin)
tar -xzvf bazel-bin/src/app_charts/victoriametrics/victoriametrics-cloudmetrics/victoriametrics-cloudmetrics-cloud-0.0.1.tgz -C /tmp

# 3. Simulate runtime values (Optional: trigger list rendering)
sed -i 's/tolerations: \[\]/tolerations:\n  - key: test/g' /tmp/victoriametrics-cloudmetrics-cloud/values.yaml

# 4. Evaluate the Go templates
helm template test-release /tmp/victoriametrics-cloudmetrics-cloud > /tmp/rendered.yaml
```

If the command exits quietly, the substitution logic produced valid YAML. If it throws a parsing error, verify your `nindent` and `trimPrefix` values match the build-time indentation exactly.

---

## Simulation of the Full Lifecycle

To illustrate the exact data flow, here is a complete lifecycle trace for the `tolerations` list substitution on the `vmstorage` component.

### 1. Build-Time Input

**File**: `core/src/app_charts/victoriametrics/victoriametrics-cloudmetrics/victoriametrics-cloudmetrics-cloud.values.yaml`

This file is fed into Bazel to construct the static Helm chart. We inject the literal bypass variable into the upstream chart schema.

```yaml
vmcluster:
  spec:
    vmstorage:
      tolerations:
        - $CR_VICTORIAMETRICS_CLOUDMETRICS_VMSTORAGE_TOLERATIONS_OBJECT: REMOVE_ME
```

### 2. Intermediate State (Post-Bazel)

**File**: `bazel-bin/src/app_charts/victoriametrics/victoriametrics-cloudmetrics/victoriametrics-cloudmetrics-cloud-0.0.1.tgz` -> Extracted as `files/victoriametrics-cloudmetrics-chart.cloud.yaml`

After `bazel build`, Helm has successfully validated the upstream chart. The output is a raw YAML file containing the unexpanded string.

```yaml
# Source: victoria-metrics-k8s-stack/templates/victoria-metrics-operator/vmcluster/vmcluster.yaml
apiVersion: operator.victoriametrics.com/v1beta1
kind: VMCluster
metadata:
  name: victoriametrics-cloudmetrics
spec:
  vmstorage:
    tolerations:
      - $CR_VICTORIAMETRICS_CLOUDMETRICS_VMSTORAGE_TOLERATIONS_OBJECT: REMOVE_ME
```

### 3. Deploy-Time Input

**File**: `core/src/app_charts/victoriametrics/victoriametrics-cloudmetrics/values-cloud.yaml`

At deploy time, a user (or `AppRollout` CRD) provides the actual Kubernetes configuration.

```yaml
vmcluster:
  spec:
    vmstorage:
      tolerations:
        - key: "workload"
          operator: "Equal"
          value: "victoriametrics"
          effect: "NoSchedule"
```

### 4. Deploy-Time Execution

**File**: `core/src/app_charts/victoriametrics/victoriametrics-cloudmetrics/cloud/victoriametrics-cloudmetrics-operator.yaml` (Executed by `chartassignment-controller`)

The operator template executes its Go `text/template` logic. It reads the intermediate YAML, formats the deploy-time input array using `toYaml | nindent 8`, and performs a string replacement.

```gotemplate
{{- $formattedTolerations := toYaml $vmstorageTolerationsList | nindent 8 -}}
{{- $data = $data | replace "- $CR_VICTORIAMETRICS_CLOUDMETRICS_VMSTORAGE_TOLERATIONS_OBJECT: REMOVE_ME" (trimPrefix "        " (trimPrefix "\n" $formattedTolerations)) -}}
```

### 5. Final Rendered Manifest

**Target**: Directly applied to the Kubernetes API server

The output string is evaluated and parsed as pure YAML, matching Kubernetes specifications flawlessly.

```yaml
apiVersion: operator.victoriametrics.com/v1beta1
kind: VMCluster
metadata:
  name: victoriametrics-cloudmetrics
spec:
  vmstorage:
    tolerations:
      - key: workload
        operator: Equal
        value: victoriametrics
        effect: NoSchedule