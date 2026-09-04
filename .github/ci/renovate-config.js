/**
 * @see https://docs.renovatebot.com/configuration-options/
 * @type {import("renovate/dist/config/types").RenovateConfig}
 */
export default {
  extends: ['config:best-practices'],
  hostRules: [
    ...(process.env.RENOVATE_GCR_TOKEN
      ? [
          {
            hostType: 'docker',
            matchHost: 'gcr.io',
            authType: 'Basic',
            token: process.env.RENOVATE_GCR_TOKEN,
          },
        ]
      : []),
  ],
  repositories: ['googlecloudrobotics/core'],
  enabledManagers: [
    'bazel-module',
    'bazelisk',
    'custom.regex',
    'dockerfile',
    'github-actions',
  ],
  customManagers: [
    {
      customType: 'regex',
      managerFilePatterns: [
        '.github/workflows/renovate.yml',
        'MODULE.bazel',
        'non_module_deps.bzl',
        '.bazelversion',
        '.github/ci/Dockerfile.integration-test-image',
      ],
      matchStrings: [
        // Match annotated versions with commit SHA digest and release tag comment, e.g.:
        //   # renovate: datasource=github-releases dep_name=golang/tools
        //   - golang.org/x/tools/cmd/goimports@24a8e95f9d7ae2696f66314da5e50c0d98ccaa90 # v0.43.0
        `(?:#|\\/\\/) renovate: datasource=(?<datasource>\\S*) dep_name=(?<depName>\\S*)(?: versioning=(?<versioning>\\S*))?\\s*[\\s-]*\\S*?(?::\\s?|\\s?=\\s?|@)(?:"|')?(?<currentDigest>[0-9a-f]{40})(?:"|')?\\s+#\\s*(?<currentValue>\\S+)`,
        // Match annotated versions.
        //
        // Discovers marked versions by looking for a `renovate:` comment. This
        // comment looks like this:
        //
        //    # renovate: datasource=github-releases dep_name=org/repo versioning=loose
        //
        // The comment may start with `#` or `//`. The `versioning`
        // specification can be omitted entirely.
        //
        // The version to update must be on the next line. It must be prefixed
        // with some kind of string.
        //
        //    some_string: 1.2.3
        //
        // The version may be separated with a `:`, `=`, `@`, or whitespace.
        // The equals sign may have whitespace on either side. The version itself
        // may be quoted in single or double quotes.
        `(?:#|\\/\\/) renovate: datasource=(?<datasource>\\S*) dep_name=(?<depName>\\S*)(?: versioning=(?<versioning>\\S*))?\\s*[\\s-]*\\S*?(?::\\s?|\\s?=\\s?|@|\\s+)(?:"|')?(?<currentValue>[^#\\s"',]+)(?:"|')?,?`,
      ],
      versioningTemplate:
        '{{#if versioning}}{{{versioning}}}{{else}}semver{{/if}}',
    },
    {
      customType: 'regex',
      managerFilePatterns: ['/(^|/)MODULE\\.bazel$/'],
      matchStrings: [
        'go_sdk\\.download\\(version\\s*=\\s*"(?<currentValue>[^"]+)"\\)',
      ],
      datasourceTemplate: 'golang-version',
      depNameTemplate: 'go',
    },
    {
      customType: 'regex',
      managerFilePatterns: ['go.mod'],
      matchStrings: ['\\ngo\\s+(?<currentValue>[0-9.]+)'],
      datasourceTemplate: 'golang-version',
      depNameTemplate: 'go',
    },
  ],
  packageRules: [
    {
      matchDatasources: ['golang-version'],
      matchDepNames: ['go'],
      groupName: 'golang',
    },
    {
      groupName: 'renovate',
      matchDepNames: ['renovate', 'ghcr.io/renovatebot/renovate'],
    },
    {
      matchDatasources: ['bazel-module'],
      matchDepNames: ['protobuf'],
      groupName: 'Protocol Buffers',
    },
    {
      matchDatasources: ['bazel-module'],
      matchDepNames: ['grpc'],
      groupName: 'gRPC',
    },
  ],

  // Get PRs for config migration
  configMigration: true,

  // Limit PR creation rate
  prHourlyLimit: 10,
  prConcurrentLimit: 50,
  branchConcurrentLimit: 0,

  reviewers: ['ensonic'],

  // Only rebase when PR is behind base branch
  rebaseWhen: 'behind-base-branch',
  keepUpdatedLabel: 'renovate-keep-updated',

  labels: ['renovate', 'dependencies'],
};

