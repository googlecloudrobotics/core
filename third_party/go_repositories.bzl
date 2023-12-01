load("@bazel_gazelle//:deps.bzl", "go_repository")

def go_repositories():
    go_repository(
        name = "cc_mvdan_sh_v3",
        importpath = "mvdan.cc/sh/v3",
        sum = "h1:hmP3UOw4f+EYexsJjFxvU38+kn+V/s2CclXHanIBkmQ=",
        version = "v3.5.1",
    )
    go_repository(
        name = "co_honnef_go_tools",
        importpath = "honnef.co/go/tools",
        sum = "h1:UoveltGrhghAA7ePc+e+QYDHXrBps2PqFZiHkGR/xK8=",
        version = "v0.0.1-2020.1.4",
    )
    go_repository(
        name = "com_github_alecthomas_kingpin_v2",
        importpath = "github.com/alecthomas/kingpin/v2",
        sum = "h1:H0aULhgmSzN8xQ3nX1uxtdlTHYoPLu5AhHxWrKI6ocU=",
        version = "v2.3.2",
    )
    go_repository(
        name = "com_github_alecthomas_template",
        importpath = "github.com/alecthomas/template",
        sum = "h1:JYp7IbQjafoB+tBA3gMyHYHrpOtNuDiK/uB5uXxq5wM=",
        version = "v0.0.0-20190718012654-fb15b899a751",
    )
    go_repository(
        name = "com_github_alecthomas_units",
        importpath = "github.com/alecthomas/units",
        sum = "h1:s6gZFSlWYmbqAuRjVTiNNhvNRfY2Wxp9nhfyel4rklc=",
        version = "v0.0.0-20211218093645-b94a6e3cc137",
    )
    go_repository(
        name = "com_github_alessio_shellescape",
        importpath = "github.com/alessio/shellescape",
        sum = "h1:V7yhSDDn8LP4lc4jS8pFkt0zCnzVJlG5JXy9BVKJUX0=",
        version = "v1.4.1",
    )
    go_repository(
        name = "com_github_antlr_antlr4_runtime_go_antlr_v4",
        importpath = "github.com/antlr/antlr4/runtime/Go/antlr/v4",
        sum = "h1:7RFfzj4SSt6nnvCPbCqijJi1nWCd+TqAT3bYCStRC18=",
        version = "v4.0.0-20230305170008-8188dc5388df",
    )
    go_repository(
        name = "com_github_armon_go_metrics",
        importpath = "github.com/armon/go-metrics",
        sum = "h1:hR91U9KYmb6bLBYLQjyM+3j+rcd/UhE+G78SFnF8gJA=",
        version = "v0.4.1",
    )
    go_repository(
        name = "com_github_armon_go_socks5",
        importpath = "github.com/armon/go-socks5",
        sum = "h1:0CwZNZbxp69SHPdPJAN/hZIm0C4OItdklCFmMRWYpio=",
        version = "v0.0.0-20160902184237-e75332964ef5",
    )
    go_repository(
        name = "com_github_asaskevich_govalidator",
        importpath = "github.com/asaskevich/govalidator",
        sum = "h1:DklsrG3dyBCFEj5IhUbnKptjxatkF07cF2ak3yi77so=",
        version = "v0.0.0-20230301143203-a9d515a09cc2",
    )
    go_repository(
        name = "com_github_aws_aws_sdk_go",
        importpath = "github.com/aws/aws-sdk-go",
        sum = "h1:c4fLlh5sLdK2DCRTY1z0hyuJZU4ygxX8m1FswL6/nF4=",
        version = "v1.45.25",
    )
    go_repository(
        name = "com_github_azure_azure_sdk_for_go_sdk_azcore",
        importpath = "github.com/Azure/azure-sdk-for-go/sdk/azcore",
        sum = "h1:9kDVnTz3vbfweTqAUmk/a/pH5pWFCHtvRpHYC0G/dcA=",
        version = "v1.8.0",
    )
    go_repository(
        name = "com_github_azure_azure_sdk_for_go_sdk_azidentity",
        importpath = "github.com/Azure/azure-sdk-for-go/sdk/azidentity",
        sum = "h1:BMAjVKJM0U/CYF27gA0ZMmXGkOcvfFtD0oHVZ1TIPRI=",
        version = "v1.4.0",
    )
    go_repository(
        name = "com_github_azure_azure_sdk_for_go_sdk_internal",
        importpath = "github.com/Azure/azure-sdk-for-go/sdk/internal",
        sum = "h1:sXr+ck84g/ZlZUOZiNELInmMgOsuGwdjjVkEIde0OtY=",
        version = "v1.3.0",
    )
    go_repository(
        name = "com_github_azure_azure_sdk_for_go_sdk_resourcemanager_compute_armcompute_v4",
        importpath = "github.com/Azure/azure-sdk-for-go/sdk/resourcemanager/compute/armcompute/v4",
        sum = "h1:UPeCRD+XY7QlaGQte2EVI2iOcWvUYA2XY8w5T/8v0NQ=",
        version = "v4.2.1",
    )
    go_repository(
        name = "com_github_azure_azure_sdk_for_go_sdk_resourcemanager_network_armnetwork",
        importpath = "github.com/Azure/azure-sdk-for-go/sdk/resourcemanager/network/armnetwork",
        sum = "h1:QM6sE5k2ZT/vI5BEe0r7mqjsUSnhVBFbOsVkEuaEfiA=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_azure_azure_sdk_for_go_sdk_resourcemanager_network_armnetwork_v2",
        importpath = "github.com/Azure/azure-sdk-for-go/sdk/resourcemanager/network/armnetwork/v2",
        sum = "h1:bWh0Z2rOEDfB/ywv/l0iHN1JgyazE6kW/aIA89+CEK0=",
        version = "v2.2.1",
    )
    go_repository(
        name = "com_github_azure_go_ansiterm",
        importpath = "github.com/Azure/go-ansiterm",
        sum = "h1:UQHMgLO+TxOElx5B5HZ4hJQsoJ/PvUvKRhJHDQXO8P8=",
        version = "v0.0.0-20210617225240-d185dfc1b5a1",
    )
    go_repository(
        name = "com_github_azuread_microsoft_authentication_library_for_go",
        importpath = "github.com/AzureAD/microsoft-authentication-library-for-go",
        sum = "h1:WpB/QDNLpMw72xHJc34BNNykqSOeEJDAWkhf0u12/Jk=",
        version = "v1.1.1",
    )
    go_repository(
        name = "com_github_bazelbuild_bazelisk",
        importpath = "github.com/bazelbuild/bazelisk",
        sum = "h1:SpigbUorngcfDULmft0WkdrYVCqqqsFPEW4hm8UA+Gk=",
        version = "v1.13.2",
    )
    go_repository(
        name = "com_github_bazelbuild_rules_go",
        importpath = "github.com/bazelbuild/rules_go",
        sum = "h1:cmObMtgIOaEU944SqXtJ9DnlS8IPGGa7pdRnsrpQzXM=",
        version = "v0.34.0",
    )
    go_repository(
        name = "com_github_benbjohnson_clock",
        importpath = "github.com/benbjohnson/clock",
        sum = "h1:Q92kusRqC1XV2MjkWETPvjJVqKetz1OzxZB7mHJLju8=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_beorn7_perks",
        importpath = "github.com/beorn7/perks",
        sum = "h1:VlbKKnNfV8bJzeqoa4cOKqO6bYr3WgKZxO8Z16+hsOM=",
        version = "v1.0.1",
    )
    go_repository(
        name = "com_github_blang_semver_v4",
        importpath = "github.com/blang/semver/v4",
        sum = "h1:1PFHFE6yCCTv8C1TeyNNarDzntLi7wMI5i/pzqYIsAM=",
        version = "v4.0.0",
    )
    go_repository(
        name = "com_github_burntsushi_toml",
        importpath = "github.com/BurntSushi/toml",
        sum = "h1:9F2/+DoOYIOksmaJFPw1tGFy1eDnIJXg+UHjuD8lTak=",
        version = "v1.2.1",
    )
    go_repository(
        name = "com_github_burntsushi_xgb",
        importpath = "github.com/BurntSushi/xgb",
        sum = "h1:1BDTz0u9nC3//pOCMdNH+CiXJVYJh5UQNCOBG7jbELc=",
        version = "v0.0.0-20160522181843-27f122750802",
    )
    go_repository(
        name = "com_github_cenkalti_backoff",
        importpath = "github.com/cenkalti/backoff",
        sum = "h1:tNowT99t7UNflLxfYYSlKYsBpXdEet03Pg2g16Swow4=",
        version = "v2.2.1+incompatible",
    )
    go_repository(
        name = "com_github_cenkalti_backoff_v4",
        importpath = "github.com/cenkalti/backoff/v4",
        sum = "h1:y4OZtCnogmCPw98Zjyt5a6+QwPLGkiQsYW5oUqylYbM=",
        version = "v4.2.1",
    )
    go_repository(
        name = "com_github_census_instrumentation_opencensus_proto",
        build_extra_args = ["-exclude=src"],  # keep
        importpath = "github.com/census-instrumentation/opencensus-proto",
        sum = "h1:iKLQ0xPNFxR/2hzXZMrBo8f1j86j5WHzznCCQxV/b8g=",
        version = "v0.4.1",
    )
    go_repository(
        name = "com_github_cespare_xxhash_v2",
        importpath = "github.com/cespare/xxhash/v2",
        sum = "h1:DC2CZ1Ep5Y4k3ZQ899DldepgrayRUGE6BBZ/cd9Cj44=",
        version = "v2.2.0",
    )
    go_repository(
        name = "com_github_chzyer_logex",
        importpath = "github.com/chzyer/logex",
        sum = "h1:Swpa1K6QvQznwJRcfTfQJmTE72DqScAa40E+fbHEXEE=",
        version = "v1.1.10",
    )
    go_repository(
        name = "com_github_chzyer_readline",
        importpath = "github.com/chzyer/readline",
        sum = "h1:fY5BOSpyZCqRo5OhCuC+XN+r/bBCmeuuJtjz+bCNIf8=",
        version = "v0.0.0-20180603132655-2972be24d48e",
    )
    go_repository(
        name = "com_github_chzyer_test",
        importpath = "github.com/chzyer/test",
        sum = "h1:q763qf9huN11kDQavWsoZXJNW3xEE4JJyHa5Q25/sd8=",
        version = "v0.0.0-20180213035817-a1ea475d72b1",
    )
    go_repository(
        name = "com_github_client9_misspell",
        importpath = "github.com/client9/misspell",
        sum = "h1:ta993UF76GwbvJcIo3Y68y/M3WxlpEHPWIGDkJYwzJI=",
        version = "v0.3.4",
    )
    go_repository(
        name = "com_github_cncf_udpa_go",
        importpath = "github.com/cncf/udpa/go",
        sum = "h1:QQ3GSy+MqSHxm/d8nCtnAiZdYFd45cYZPs8vOOIYKfk=",
        version = "v0.0.0-20220112060539-c52dc94e7fbe",
    )
    go_repository(
        name = "com_github_cncf_xds_go",
        importpath = "github.com/cncf/xds/go",
        sum = "h1:/inchEIKaYC1Akx+H+gqO04wryn5h75LSazbRlnya1k=",
        version = "v0.0.0-20230607035331-e9ce68804cb4",
    )
    go_repository(
        name = "com_github_coreos_go_semver",
        importpath = "github.com/coreos/go-semver",
        sum = "h1:yi21YpKnrx1gt5R+la8n5WgS0kCrsPp33dmEyHReZr4=",
        version = "v0.3.1",
    )
    go_repository(
        name = "com_github_coreos_go_systemd_v22",
        importpath = "github.com/coreos/go-systemd/v22",
        sum = "h1:RrqgGjYQKalulkV8NGVIfkXQf6YYmOyiJKk8iXXhfZs=",
        version = "v22.5.0",
    )
    go_repository(
        name = "com_github_cpuguy83_go_md2man_v2",
        importpath = "github.com/cpuguy83/go-md2man/v2",
        sum = "h1:qMCsGGgs+MAzDFyp9LpAe1Lqy/fY/qCovCm0qnXZOBM=",
        version = "v2.0.3",
    )
    go_repository(
        name = "com_github_creack_pty",
        importpath = "github.com/creack/pty",
        sum = "h1:uDmaGzcdjhF4i/plgjmEsriH11Y0o7RKapEf/LDaM3w=",
        version = "v1.1.9",
    )
    go_repository(
        name = "com_github_cyphar_filepath_securejoin",
        importpath = "github.com/cyphar/filepath-securejoin",
        sum = "h1:Ugdm7cg7i6ZK6x3xDF1oEu1nfkyfH53EtKeQYTC3kyg=",
        version = "v0.2.4",
    )
    go_repository(
        name = "com_github_davecgh_go_spew",
        importpath = "github.com/davecgh/go-spew",
        sum = "h1:U9qPSI2PIWSS1VwoXQT9A3Wy9MM3WgvqSxFWenqJduM=",
        version = "v1.1.2-0.20180830191138-d8f796af33cc",
    )
    go_repository(
        name = "com_github_dennwc_varint",
        importpath = "github.com/dennwc/varint",
        sum = "h1:kGNFFSSw8ToIy3obO/kKr8U9GZYUAxQEVuix4zfDWzE=",
        version = "v1.0.0",
    )
    go_repository(
        name = "com_github_digitalocean_godo",
        importpath = "github.com/digitalocean/godo",
        sum = "h1:SZNxjAsskM/su0YW9P8Wx3gU0W1Z13b6tZlYNpl5BnA=",
        version = "v1.104.1",
    )
    go_repository(
        name = "com_github_docker_distribution",
        importpath = "github.com/docker/distribution",
        sum = "h1:T3de5rq0dB1j30rp0sA2rER+m322EBzniBPB6ZIzuh8=",
        version = "v2.8.2+incompatible",
    )
    go_repository(
        name = "com_github_docker_docker",
        importpath = "github.com/docker/docker",
        sum = "h1:hceabKCtUgDqPu+qm0NgsaXf28Ljf4/pWFL7xjWWDgE=",
        version = "v24.0.6+incompatible",
    )
    go_repository(
        name = "com_github_docker_go_connections",
        importpath = "github.com/docker/go-connections",
        sum = "h1:El9xVISelRB7BuFusrZozjnkIM5YnzCViNKohAFqRJQ=",
        version = "v0.4.0",
    )
    go_repository(
        name = "com_github_docker_go_units",
        importpath = "github.com/docker/go-units",
        sum = "h1:69rxXcBk27SvSaaxTtLh/8llcHD8vYHT7WSdRZ/jvr4=",
        version = "v0.5.0",
    )
    go_repository(
        name = "com_github_dustin_go_humanize",
        importpath = "github.com/dustin/go-humanize",
        sum = "h1:GzkhY7T5VNhEkwH0PVJgjz+fX1rhBrR7pRT3mDkpeCY=",
        version = "v1.0.1",
    )
    go_repository(
        name = "com_github_edsrzf_mmap_go",
        importpath = "github.com/edsrzf/mmap-go",
        sum = "h1:6EUwBLQ/Mcr1EYLE4Tn1VdW1A4ckqCQWZBw8Hr0kjpQ=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_emicklei_go_restful_v3",
        importpath = "github.com/emicklei/go-restful/v3",
        sum = "h1:rAQeMHw1c7zTmncogyy8VvRZwtkmkZ4FxERmMY4rD+g=",
        version = "v3.11.0",
    )
    go_repository(
        name = "com_github_envoyproxy_go_control_plane",
        importpath = "github.com/envoyproxy/go-control-plane",
        sum = "h1:wSUXTlLfiAQRWs2F+p+EKOY9rUyis1MyGqJ2DIk5HpM=",
        version = "v0.11.1",
    )
    go_repository(
        name = "com_github_envoyproxy_protoc_gen_validate",
        importpath = "github.com/envoyproxy/protoc-gen-validate",
        sum = "h1:QkIBuU5k+x7/QXPvPPnWXWlCdaBFApVqftFV6k087DA=",
        version = "v1.0.2",
    )
    go_repository(
        name = "com_github_evanphx_json_patch",
        importpath = "github.com/evanphx/json-patch",
        sum = "h1:vgGkfT/9f8zE6tvSCe74nfpAVDQ2tG6yudJd8LBksgI=",
        version = "v5.7.0+incompatible",
    )
    go_repository(
        name = "com_github_evanphx_json_patch_v5",
        importpath = "github.com/evanphx/json-patch/v5",
        sum = "h1:nJqP7uwL84RJInrohHfW0Fx3awjbm8qZeFv0nW9SYGc=",
        version = "v5.7.0",
    )
    go_repository(
        name = "com_github_fatih_color",
        importpath = "github.com/fatih/color",
        sum = "h1:kOqh6YHBtK8aywxGerMG2Eq3H6Qgoqeo13Bk2Mv/nBs=",
        version = "v1.15.0",
    )
    go_repository(
        name = "com_github_felixge_httpsnoop",
        importpath = "github.com/felixge/httpsnoop",
        sum = "h1:s/nj+GCswXYzN5v2DpNMuMQYe+0DDwt5WVCU6CWBdXk=",
        version = "v1.0.3",
    )
    go_repository(
        name = "com_github_form3tech_oss_jwt_go",
        importpath = "github.com/form3tech-oss/jwt-go",
        sum = "h1:/l4kBbb4/vGSsdtB5nUe8L7B9mImVMaBPw9L/0TBHU8=",
        version = "v3.2.5+incompatible",
    )
    go_repository(
        name = "com_github_fsnotify_fsnotify",
        importpath = "github.com/fsnotify/fsnotify",
        sum = "h1:8JEhPFa5W2WU7YfeZzPNqzMP6Lwt7L2715Ggo0nosvA=",
        version = "v1.7.0",
    )
    go_repository(
        name = "com_github_getlantern_httptest",
        importpath = "github.com/getlantern/httptest",
        sum = "h1:OhyiFx+yBN30O3IHrIq+9LAEhy6o7fin21wUQxF8NiE=",
        version = "v0.0.0-20161025015934-4b40f4c7e590",
    )
    go_repository(
        name = "com_github_getlantern_mockconn",
        importpath = "github.com/getlantern/mockconn",
        sum = "h1:+aO65ByJw74kV8vXqvkj49P5RtIqyUObyeRTIxMz218=",
        version = "v0.0.0-20190403061815-a8ffa60494a6",
    )
    go_repository(
        name = "com_github_ghodss_yaml",
        importpath = "github.com/ghodss/yaml",
        sum = "h1:wQHKEahhL6wmXdzwWG11gIVCkOv05bNOh+Rxn0yngAk=",
        version = "v1.0.0",
    )
    go_repository(
        name = "com_github_go_errors_errors",
        importpath = "github.com/go-errors/errors",
        sum = "h1:J6MZopCL4uSllY1OfXM374weqZFFItUbrImctkmUxIA=",
        version = "v1.4.2",
    )
    go_repository(
        name = "com_github_go_gl_glfw",
        importpath = "github.com/go-gl/glfw",
        sum = "h1:QbL/5oDUmRBzO9/Z7Seo6zf912W/a6Sr4Eu0G/3Jho0=",
        version = "v0.0.0-20190409004039-e6da0acd62b1",
    )
    go_repository(
        name = "com_github_go_gl_glfw_v3_3_glfw",
        importpath = "github.com/go-gl/glfw/v3.3/glfw",
        sum = "h1:WtGNWLvXpe6ZudgnXrq0barxBImvnnJoMEhXAzcbM0I=",
        version = "v0.0.0-20200222043503-6f7a984d4dc4",
    )
    go_repository(
        name = "com_github_go_kit_kit",
        importpath = "github.com/go-kit/kit",
        sum = "h1:e4o3o3IsBfAKQh5Qbbiqyfu97Ku7jrO/JbohvztANh4=",
        version = "v0.12.0",
    )
    go_repository(
        name = "com_github_go_kit_log",
        importpath = "github.com/go-kit/log",
        sum = "h1:MRVx0/zhvdseW+Gza6N9rVzU/IVzaeE1SFI4raAhmBU=",
        version = "v0.2.1",
    )
    go_repository(
        name = "com_github_go_logfmt_logfmt",
        importpath = "github.com/go-logfmt/logfmt",
        sum = "h1:wGYYu3uicYdqXVgoYbvnkrPVXkuLM1p1ifugDMEdRi4=",
        version = "v0.6.0",
    )
    go_repository(
        name = "com_github_go_logr_logr",
        importpath = "github.com/go-logr/logr",
        sum = "h1:2y3SDp0ZXuc6/cjLSZ+Q3ir+QB9T/iG5yYRXqsagWSY=",
        version = "v1.3.0",
    )
    go_repository(
        name = "com_github_go_logr_stdr",
        importpath = "github.com/go-logr/stdr",
        sum = "h1:hSWxHoqTgW2S2qGc0LTAI563KZ5YKYRhT3MFKZMbjag=",
        version = "v1.2.2",
    )
    go_repository(
        name = "com_github_go_logr_zapr",
        importpath = "github.com/go-logr/zapr",
        sum = "h1:QHVo+6stLbfJmYGkQ7uGHUCu5hnAFAj6mDe6Ea0SeOo=",
        version = "v1.2.4",
    )
    go_repository(
        name = "com_github_go_openapi_analysis",
        importpath = "github.com/go-openapi/analysis",
        sum = "h1:ZDFLvSNxpDaomuCueM0BlSXxpANBlFYiBvr+GXrvIHc=",
        version = "v0.21.4",
    )
    go_repository(
        name = "com_github_go_openapi_errors",
        importpath = "github.com/go-openapi/errors",
        sum = "h1:unTcVm6PispJsMECE3zWgvG4xTiKda1LIR5rCRWLG6M=",
        version = "v0.20.4",
    )
    go_repository(
        name = "com_github_go_openapi_jsonpointer",
        importpath = "github.com/go-openapi/jsonpointer",
        sum = "h1:ESKJdU9ASRfaPNOPRx12IUyA1vn3R9GiE3KYD14BXdQ=",
        version = "v0.20.0",
    )
    go_repository(
        name = "com_github_go_openapi_jsonreference",
        importpath = "github.com/go-openapi/jsonreference",
        sum = "h1:3sVjiK66+uXK/6oQ8xgcRKcFgQ5KXa2KvnJRumpMGbE=",
        version = "v0.20.2",
    )
    go_repository(
        name = "com_github_go_openapi_loads",
        importpath = "github.com/go-openapi/loads",
        sum = "h1:r2a/xFIYeZ4Qd2TnGpWDIQNcP80dIaZgf704za8enro=",
        version = "v0.21.2",
    )
    go_repository(
        name = "com_github_go_openapi_spec",
        importpath = "github.com/go-openapi/spec",
        sum = "h1:xnlYNQAwKd2VQRRfwTEI0DcK+2cbuvI/0c7jx3gA8/8=",
        version = "v0.20.9",
    )
    go_repository(
        name = "com_github_go_openapi_strfmt",
        importpath = "github.com/go-openapi/strfmt",
        sum = "h1:rspiXgNWgeUzhjo1YU01do6qsahtJNByjLVbPLNHb8k=",
        version = "v0.21.7",
    )
    go_repository(
        name = "com_github_go_openapi_swag",
        importpath = "github.com/go-openapi/swag",
        sum = "h1:QLMzNJnMGPRNDCbySlcj1x01tzU8/9LTTL9hZZZogBU=",
        version = "v0.22.4",
    )
    go_repository(
        name = "com_github_go_openapi_validate",
        importpath = "github.com/go-openapi/validate",
        sum = "h1:G+c2ub6q47kfX1sOBLwIQwzBVt8qmOAARyo/9Fqs9NU=",
        version = "v0.22.1",
    )
    go_repository(
        name = "com_github_go_resty_resty_v2",
        importpath = "github.com/go-resty/resty/v2",
        sum = "h1:me+K9p3uhSmXtrBZ4k9jcEAfJmuC8IivWHwaLZwPrFY=",
        version = "v2.7.0",
    )
    go_repository(
        name = "com_github_go_stack_stack",
        importpath = "github.com/go-stack/stack",
        sum = "h1:5SgMzNM5HxrEjV0ww2lTmX6E2Izsfxas4+YHWRs3Lsk=",
        version = "v1.8.0",
    )
    go_repository(
        name = "com_github_go_task_slim_sprig",
        importpath = "github.com/go-task/slim-sprig",
        sum = "h1:tfuBGBXKqDEevZMzYi5KSi8KkcZtzBcTgAUUtapy0OI=",
        version = "v0.0.0-20230315185526-52ccab3ef572",
    )
    go_repository(
        name = "com_github_go_zookeeper_zk",
        importpath = "github.com/go-zookeeper/zk",
        sum = "h1:7M2kwOsc//9VeeFiPtf+uSJlVpU66x9Ba5+8XK7/TDg=",
        version = "v1.0.3",
    )
    go_repository(
        name = "com_github_gobwas_glob",
        importpath = "github.com/gobwas/glob",
        sum = "h1:A4xDbljILXROh+kObIiy5kIaPYD8e96x1tgBhUI5J+Y=",
        version = "v0.2.3",
    )
    go_repository(
        name = "com_github_gogo_protobuf",
        build_file_proto_mode = "disable",
        importpath = "github.com/gogo/protobuf",
        sum = "h1:Ov1cvc58UF3b5XjBnZv7+opcTcQFZebYjWzi34vdm4Q=",
        version = "v1.3.2",
    )
    go_repository(
        name = "com_github_golang_glog",
        importpath = "github.com/golang/glog",
        sum = "h1:DVjP2PbBOzHyzA+dn3WhHIq4NdVu3Q+pvivFICf/7fo=",
        version = "v1.1.2",
    )
    go_repository(
        name = "com_github_golang_groupcache",
        importpath = "github.com/golang/groupcache",
        sum = "h1:oI5xCqsCo564l8iNU+DwB5epxmsaqB+rhGL0m5jtYqE=",
        version = "v0.0.0-20210331224755-41bb18bfe9da",
    )
    go_repository(
        name = "com_github_golang_jwt_jwt_v4",
        importpath = "github.com/golang-jwt/jwt/v4",
        sum = "h1:7cYmW1XlMY7h7ii7UhUyChSgS5wUJEnm9uZVTGqOWzg=",
        version = "v4.5.0",
    )
    go_repository(
        name = "com_github_golang_jwt_jwt_v5",
        importpath = "github.com/golang-jwt/jwt/v5",
        sum = "h1:1n1XNM9hk7O9mnQoNBGolZvzebBQ7p93ULHRc28XJUE=",
        version = "v5.0.0",
    )
    go_repository(
        name = "com_github_golang_mock",
        importpath = "github.com/golang/mock",
        sum = "h1:ErTB+efbowRARo13NNdxyJji2egdxLGQhRaY+DUumQc=",
        version = "v1.6.0",
    )
    go_repository(
        name = "com_github_golang_protobuf",
        importpath = "github.com/golang/protobuf",
        sum = "h1:KhyjKVUg7Usr/dYsdSqoFveMYd5ko72D+zANwlG1mmg=",
        version = "v1.5.3",
    )
    go_repository(
        name = "com_github_golang_snappy",
        importpath = "github.com/golang/snappy",
        sum = "h1:yAGX7huGHXlcLOEtBnF4w7FQwA26wojNCwOYAEhLjQM=",
        version = "v0.0.4",
    )
    go_repository(
        name = "com_github_google_btree",
        importpath = "github.com/google/btree",
        sum = "h1:xf4v41cLI2Z6FxbKm+8Bu+m8ifhj15JuZ9sa0jZCMUU=",
        version = "v1.1.2",
    )
    go_repository(
        name = "com_github_google_cel_go",
        importpath = "github.com/google/cel-go",
        sum = "h1:3hZfSNiAU3KOiNtxuFXVp5WFy4hf/Ly3Sa4/7F8SXNo=",
        version = "v0.16.1",
    )
    go_repository(
        name = "com_github_google_gnostic_models",
        build_file_proto_mode = "disable",
        importpath = "github.com/google/gnostic-models",
        sum = "h1:0VpGH+cDhbDtdcweoyCVsF3fhN8kejK6rFe/2FFX2nU=",
        version = "v0.6.9-0.20230804172637-c7be7c783f49",
    )
    go_repository(
        name = "com_github_google_go_cmp",
        importpath = "github.com/google/go-cmp",
        sum = "h1:ofyhxvXcZhMsU5ulbFiLKl/XBFqE1GSq7atu8tAmTRI=",
        version = "v0.6.0",
    )
    go_repository(
        name = "com_github_google_go_pkcs11",
        importpath = "github.com/google/go-pkcs11",
        sum = "h1:OF1IPgv+F4NmqmJ98KTjdN97Vs1JxDPB3vbmYzV2dpk=",
        version = "v0.2.1-0.20230907215043-c6f79328ddf9",
    )
    go_repository(
        name = "com_github_google_go_querystring",
        importpath = "github.com/google/go-querystring",
        sum = "h1:AnCroh3fv4ZBgVIf1Iwtovgjaw/GiKJo8M8yD/fhyJ8=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_google_gofuzz",
        importpath = "github.com/google/gofuzz",
        sum = "h1:xRy4A+RhZaiKjJ1bPfwQ8sedCA+YS2YcCHW6ec7JMi0=",
        version = "v1.2.0",
    )
    go_repository(
        name = "com_github_google_martian",
        importpath = "github.com/google/martian",
        sum = "h1:/CP5g8u/VJHijgedC/Legn3BAbAaWPgecwXBIDzw5no=",
        version = "v2.1.0+incompatible",
    )
    go_repository(
        name = "com_github_google_martian_v3",
        importpath = "github.com/google/martian/v3",
        sum = "h1:IqNFLAmvJOgVlpdEBiQbDc2EwKW77amAycfTuWKdfvw=",
        version = "v3.3.2",
    )
    go_repository(
        name = "com_github_google_pprof",
        importpath = "github.com/google/pprof",
        sum = "h1:pUa4ghanp6q4IJHwE9RwLgmVFfReJN+KbQ8ExNEUUoQ=",
        version = "v0.0.0-20230926050212-f7f687d19a98",
    )
    go_repository(
        name = "com_github_google_renameio",
        importpath = "github.com/google/renameio",
        sum = "h1:GOZbcHa3HfsPKPlmyPyN2KEohoMXOhdMbHrvbpl2QaA=",
        version = "v0.1.0",
    )
    go_repository(
        name = "com_github_google_s2a_go",
        importpath = "github.com/google/s2a-go",
        sum = "h1:60BLSyTrOV4/haCDW4zb1guZItoSq8foHCXrAnjBo/o=",
        version = "v0.1.7",
    )
    go_repository(
        name = "com_github_google_safetext",
        importpath = "github.com/google/safetext",
        sum = "h1:03r+JaAB8/2z83KOOCZK95tslx6e41NZS4Tpt569MtY=",
        version = "v0.0.0-20221026122733-23539d61753f",
    )
    go_repository(
        name = "com_github_google_shlex",
        importpath = "github.com/google/shlex",
        sum = "h1:El6M4kTTCOh6aBiKaUGG7oYTSPP8MxqL4YI3kZKwcP4=",
        version = "v0.0.0-20191202100458-e7afc7fbc510",
    )
    go_repository(
        name = "com_github_google_uuid",
        importpath = "github.com/google/uuid",
        sum = "h1:MtMxsa51/r9yyhkyLsVeVt0B+BGQZzpQiTQ4eHZ8bc4=",
        version = "v1.4.0",
    )
    go_repository(
        name = "com_github_googleapis_enterprise_certificate_proxy",
        importpath = "github.com/googleapis/enterprise-certificate-proxy",
        sum = "h1:Vie5ybvEvT75RniqhfFxPRy3Bf7vr3h0cechB90XaQs=",
        version = "v0.3.2",
    )
    go_repository(
        name = "com_github_googleapis_gax_go_v2",
        build_file_proto_mode = "disable_global",
        importpath = "github.com/googleapis/gax-go/v2",
        sum = "h1:A+gCJKdRfqXkr+BIRGtZLibNXf0m1f9E4HG56etFpas=",
        version = "v2.12.0",
    )
    go_repository(
        name = "com_github_googlecloudrobotics_ilog",
        importpath = "github.com/googlecloudrobotics/ilog",
        sum = "h1:XEQxKohGzFEBOj5O2fcR1+gxDEgznOdb6G34kdG49u8=",
        version = "v0.0.0-20231123101900-2533d3d7f8e1",
    )
    go_repository(
        name = "com_github_gophercloud_gophercloud",
        importpath = "github.com/gophercloud/gophercloud",
        sum = "h1:fyJGKh0LBvIZKLvBWvQdIgkaV5yTM3Jh9EYUh+UNCAs=",
        version = "v1.7.0",
    )
    go_repository(
        name = "com_github_gorilla_websocket",
        importpath = "github.com/gorilla/websocket",
        sum = "h1:PPwGk2jz7EePpoHN/+ClbZu8SPxiqlu12wZP/3sWmnc=",
        version = "v1.5.0",
    )
    go_repository(
        name = "com_github_grafana_regexp",
        importpath = "github.com/grafana/regexp",
        sum = "h1:PpuIBO5P3e9hpqBD0O/HjhShYuM6XE0i/lbE6J94kww=",
        version = "v0.0.0-20221122212121-6b5c0a4cb7fd",
    )
    go_repository(
        name = "com_github_gregjones_httpcache",
        importpath = "github.com/gregjones/httpcache",
        sum = "h1:+ngKgrYPPJrOjhax5N+uePQ0Fh1Z7PheYoUI/0nzkPA=",
        version = "v0.0.0-20190611155906-901d90724c79",
    )
    go_repository(
        name = "com_github_grpc_ecosystem_go_grpc_middleware",
        importpath = "github.com/grpc-ecosystem/go-grpc-middleware",
        sum = "h1:+9834+KizmvFV7pXQGSXQTsaWhq2GjuNUt0aUU0YBYw=",
        version = "v1.3.0",
    )
    go_repository(
        name = "com_github_grpc_ecosystem_go_grpc_prometheus",
        importpath = "github.com/grpc-ecosystem/go-grpc-prometheus",
        sum = "h1:Ovs26xHkKqVztRpIrF/92BcuyuQ/YW4NSIpoGtfXNho=",
        version = "v1.2.0",
    )
    go_repository(
        name = "com_github_grpc_ecosystem_grpc_gateway",
        importpath = "github.com/grpc-ecosystem/grpc-gateway",
        sum = "h1:gmcG1KaJ57LophUzW0Hy8NmPhnMZb4M0+kPpLofRdBo=",
        version = "v1.16.0",
    )
    go_repository(
        name = "com_github_grpc_ecosystem_grpc_gateway_v2",
        importpath = "github.com/grpc-ecosystem/grpc-gateway/v2",
        sum = "h1:YBftPWNWd4WwGqtY2yeZL2ef8rHAxPBD8KFhJpmcqms=",
        version = "v2.16.0",
    )
    go_repository(
        name = "com_github_h2non_parth",
        importpath = "github.com/h2non/parth",
        sum = "h1:2VTzZjLZBgl62/EtslCrtky5vbi9dd7HrQPQIx6wqiw=",
        version = "v0.0.0-20190131123155-b4df798d6542",
    )
    go_repository(
        name = "com_github_hashicorp_consul_api",
        importpath = "github.com/hashicorp/consul/api",
        sum = "h1:CqrdhYzc8XZuPnhIYZWH45toM0LB9ZeYr/gvpLVI3PE=",
        version = "v1.25.1",
    )
    go_repository(
        name = "com_github_hashicorp_cronexpr",
        importpath = "github.com/hashicorp/cronexpr",
        sum = "h1:wG/ZYIKT+RT3QkOdgYc+xsKWVRgnxJ1OJtjjy84fJ9A=",
        version = "v1.1.2",
    )
    go_repository(
        name = "com_github_hashicorp_errwrap",
        importpath = "github.com/hashicorp/errwrap",
        sum = "h1:OxrOeh75EUXMY8TBjag2fzXGZ40LB6IKw45YeGUDY2I=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_hashicorp_go_cleanhttp",
        importpath = "github.com/hashicorp/go-cleanhttp",
        sum = "h1:035FKYIWjmULyFRBKPs8TBQoi0x6d9G4xc9neXJWAZQ=",
        version = "v0.5.2",
    )
    go_repository(
        name = "com_github_hashicorp_go_hclog",
        importpath = "github.com/hashicorp/go-hclog",
        sum = "h1:bI2ocEMgcVlz55Oj1xZNBsVi900c7II+fWDyV9o+13c=",
        version = "v1.5.0",
    )
    go_repository(
        name = "com_github_hashicorp_go_immutable_radix",
        importpath = "github.com/hashicorp/go-immutable-radix",
        sum = "h1:DKHmCUm2hRBK510BaiZlwvpD40f8bJFeZnpfm2KLowc=",
        version = "v1.3.1",
    )
    go_repository(
        name = "com_github_hashicorp_go_multierror",
        importpath = "github.com/hashicorp/go-multierror",
        sum = "h1:H5DkEtf6CXdFp0N0Em5UCwQpXMWke8IA0+lD48awMYo=",
        version = "v1.1.1",
    )
    go_repository(
        name = "com_github_hashicorp_go_retryablehttp",
        importpath = "github.com/hashicorp/go-retryablehttp",
        sum = "h1:ZQgVdpTdAL7WpMIwLzCfbalOcSUdkDZnpUv3/+BxzFA=",
        version = "v0.7.4",
    )
    go_repository(
        name = "com_github_hashicorp_go_rootcerts",
        importpath = "github.com/hashicorp/go-rootcerts",
        sum = "h1:jzhAVGtqPKbwpyCPELlgNWhE1znq+qwJtW5Oi2viEzc=",
        version = "v1.0.2",
    )
    go_repository(
        name = "com_github_hashicorp_golang_lru",
        importpath = "github.com/hashicorp/golang-lru",
        sum = "h1:uL2shRDx7RTrOrTCUZEGP/wJUFiUI8QT6E7z5o8jga4=",
        version = "v0.6.0",
    )
    go_repository(
        name = "com_github_hashicorp_nomad_api",
        importpath = "github.com/hashicorp/nomad/api",
        sum = "h1:Nc3Mt2BAnq0/VoLEntF/nipX+K1S7pG+RgwiitSv6v0=",
        version = "v0.0.0-20230721134942-515895c7690c",
    )
    go_repository(
        name = "com_github_hashicorp_serf",
        importpath = "github.com/hashicorp/serf",
        sum = "h1:Z1H2J60yRKvfDYAOZLd2MU0ND4AH/WDz7xYHDWQsIPY=",
        version = "v0.10.1",
    )
    go_repository(
        name = "com_github_hetznercloud_hcloud_go_v2",
        importpath = "github.com/hetznercloud/hcloud-go/v2",
        sum = "h1:MqlAE+w125PLvJRCpAJmEwrIxoVdUdOyuFUhE/Ukbok=",
        version = "v2.4.0",
    )
    go_repository(
        name = "com_github_huandu_xstrings",
        importpath = "github.com/huandu/xstrings",
        sum = "h1:/Gcsuc1x8JVbJ9/rlye4xZnVAbEkGauT8lbebqcQws4=",
        version = "v1.3.3",
    )
    go_repository(
        name = "com_github_ianlancetaylor_demangle",
        importpath = "github.com/ianlancetaylor/demangle",
        sum = "h1:UDMh68UUwekSh5iP2OMhRRZJiiBccgV7axzUG8vi56c=",
        version = "v0.0.0-20181102032728-5e5cf60278f6",
    )
    go_repository(
        name = "com_github_imdario_mergo",
        importpath = "github.com/imdario/mergo",
        sum = "h1:wwQJbIsHYGMUyLSPrEq1CT16AhnhNJQ51+4fdHUnCl4=",
        version = "v0.3.16",
    )
    go_repository(
        name = "com_github_inconshreveable_mousetrap",
        importpath = "github.com/inconshreveable/mousetrap",
        sum = "h1:wN+x4NVGpMsO7ErUn/mUI3vEoE6Jt13X2s0bqwp9tc8=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_ionos_cloud_sdk_go_v6",
        importpath = "github.com/ionos-cloud/sdk-go/v6",
        sum = "h1:Iq3VIXzeEbc8EbButuACgfLMiY5TPVWUPNrF+Vsddo4=",
        version = "v6.1.9",
    )
    go_repository(
        name = "com_github_jessevdk_go_flags",
        importpath = "github.com/jessevdk/go-flags",
        sum = "h1:4IU2WS7AumrZ/40jfhf4QVDMsQwqA7VEHozFRrGARJA=",
        version = "v1.4.0",
    )
    go_repository(
        name = "com_github_jmespath_go_jmespath",
        importpath = "github.com/jmespath/go-jmespath",
        sum = "h1:BEgLn5cpjn8UN1mAw4NjwDrS35OdebyEtFe+9YPoQUg=",
        version = "v0.4.0",
    )
    go_repository(
        name = "com_github_jmespath_go_jmespath_internal_testify",
        importpath = "github.com/jmespath/go-jmespath/internal/testify",
        sum = "h1:shLQSRRSCCPj3f2gpwzGwWFoC7ycTf1rcQZHOlsJ6N8=",
        version = "v1.5.1",
    )
    go_repository(
        name = "com_github_jonboulle_clockwork",
        importpath = "github.com/jonboulle/clockwork",
        sum = "h1:UOGuzwb1PwsrDAObMuhUnj0p5ULPj8V/xJ7Kx9qUBdQ=",
        version = "v0.2.2",
    )
    go_repository(
        name = "com_github_josharian_intern",
        importpath = "github.com/josharian/intern",
        sum = "h1:vlS4z54oSdjm0bgjRigI+G1HpF+tI+9rE5LLzOg8HmY=",
        version = "v1.0.0",
    )
    go_repository(
        name = "com_github_jpillora_backoff",
        importpath = "github.com/jpillora/backoff",
        sum = "h1:uvFg412JmmHBHw7iwprIxkPMI+sGQ4kzOWsMeHnm2EA=",
        version = "v1.0.0",
    )
    go_repository(
        name = "com_github_json_iterator_go",
        importpath = "github.com/json-iterator/go",
        sum = "h1:PV8peI4a0ysnczrg+LtxykD8LfKY9ML6u2jnxaEnrnM=",
        version = "v1.1.12",
    )
    go_repository(
        name = "com_github_jstemmer_go_junit_report",
        importpath = "github.com/jstemmer/go-junit-report",
        sum = "h1:6QPYqodiu3GuPL+7mfx+NwDdp2eTkp9IfEUpgAwUN0o=",
        version = "v0.9.1",
    )
    go_repository(
        name = "com_github_julienschmidt_httprouter",
        importpath = "github.com/julienschmidt/httprouter",
        sum = "h1:U0609e9tgbseu3rBINet9P48AI/D3oJs4dN7jwJOQ1U=",
        version = "v1.3.0",
    )
    go_repository(
        name = "com_github_kisielk_errcheck",
        importpath = "github.com/kisielk/errcheck",
        sum = "h1:e8esj/e4R+SAOwFwN+n3zr0nYeCyeweozKfO23MvHzY=",
        version = "v1.5.0",
    )
    go_repository(
        name = "com_github_kisielk_gotool",
        importpath = "github.com/kisielk/gotool",
        sum = "h1:AV2c/EiW3KqPNT9ZKl07ehoAGi4C5/01Cfbblndcapg=",
        version = "v1.0.0",
    )
    go_repository(
        name = "com_github_klauspost_compress",
        importpath = "github.com/klauspost/compress",
        sum = "h1:NE3C767s2ak2bweCZo3+rdP4U/HoyVXLv/X9f2gPS5g=",
        version = "v1.17.1",
    )
    go_repository(
        name = "com_github_kolo_xmlrpc",
        importpath = "github.com/kolo/xmlrpc",
        sum = "h1:udzkj9S/zlT5X367kqJis0QP7YMxobob6zhzq6Yre00=",
        version = "v0.0.0-20220921171641-a4b6fa1dd06b",
    )
    go_repository(
        name = "com_github_konsorten_go_windows_terminal_sequences",
        importpath = "github.com/konsorten/go-windows-terminal-sequences",
        sum = "h1:CE8S1cTafDpPvMhIxNJKvHsGVBgn1xWYf1NbHQhywc8=",
        version = "v1.0.3",
    )
    go_repository(
        name = "com_github_kr_logfmt",
        importpath = "github.com/kr/logfmt",
        sum = "h1:T+h1c/A9Gawja4Y9mFVWj2vyii2bbUNDw3kt9VxK2EY=",
        version = "v0.0.0-20140226030751-b84e30acd515",
    )
    go_repository(
        name = "com_github_kr_pretty",
        importpath = "github.com/kr/pretty",
        sum = "h1:flRD4NNwYAUpkphVc1HcthR4KEIFJ65n8Mw5qdRn3LE=",
        version = "v0.3.1",
    )
    go_repository(
        name = "com_github_kr_pty",
        importpath = "github.com/kr/pty",
        sum = "h1:VkoXIwSboBpnk99O/KFauAEILuNHv5DVFKZMBN/gUgw=",
        version = "v1.1.1",
    )
    go_repository(
        name = "com_github_kr_text",
        importpath = "github.com/kr/text",
        sum = "h1:5Nx0Ya0ZqY2ygV366QzturHI13Jq95ApcVaJBhpS+AY=",
        version = "v0.2.0",
    )
    go_repository(
        name = "com_github_kylelemons_godebug",
        importpath = "github.com/kylelemons/godebug",
        sum = "h1:RPNrshWIDI6G2gRW9EHilWtl7Z6Sb1BR0xunSBf0SNc=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_liggitt_tabwriter",
        importpath = "github.com/liggitt/tabwriter",
        sum = "h1:9TO3cAIGXtEhnIaL+V+BEER86oLrvS+kWobKpbJuye0=",
        version = "v0.0.0-20181228230101-89fcab3d43de",
    )
    go_repository(
        name = "com_github_linode_linodego",
        importpath = "github.com/linode/linodego",
        sum = "h1:s0ReCZtuN9Z1IoUN9w1RLeYO1dMZUGPwOQ/IBFsBHtU=",
        version = "v1.23.0",
    )
    go_repository(
        name = "com_github_mailru_easyjson",
        importpath = "github.com/mailru/easyjson",
        sum = "h1:UGYAvKxe3sBsEDzO8ZeWOSlIQfWFlxbzLZe7hwFURr0=",
        version = "v0.7.7",
    )
    go_repository(
        name = "com_github_masterminds_goutils",
        importpath = "github.com/Masterminds/goutils",
        sum = "h1:5nUrii3FMTL5diU80unEVvNevw1nH4+ZV4DSLVJLSYI=",
        version = "v1.1.1",
    )
    go_repository(
        name = "com_github_masterminds_semver",
        importpath = "github.com/Masterminds/semver",
        sum = "h1:H65muMkzWKEuNDnfl9d70GUjFniHKHRbFPGBuZ3QEww=",
        version = "v1.5.0",
    )
    go_repository(
        name = "com_github_masterminds_sprig",
        importpath = "github.com/Masterminds/sprig",
        sum = "h1:z4yfnGrZ7netVz+0EDJ0Wi+5VZCSYp4Z0m2dk6cEM60=",
        version = "v2.22.0+incompatible",
    )
    go_repository(
        name = "com_github_mattn_go_colorable",
        importpath = "github.com/mattn/go-colorable",
        sum = "h1:fFA4WZxdEF4tXPZVKMLwD8oUnCTTo08duU7wxecdEvA=",
        version = "v0.1.13",
    )
    go_repository(
        name = "com_github_mattn_go_isatty",
        importpath = "github.com/mattn/go-isatty",
        sum = "h1:JITubQf0MOLdlGRuRq+jtsDlekdYPia9ZFsB8h/APPA=",
        version = "v0.0.19",
    )
    go_repository(
        name = "com_github_matttproud_golang_protobuf_extensions",
        importpath = "github.com/matttproud/golang_protobuf_extensions",
        sum = "h1:mmDVorXM7PCGKw94cs5zkfA9PSy5pEvNWRP0ET0TIVo=",
        version = "v1.0.4",
    )
    go_repository(
        name = "com_github_matttproud_golang_protobuf_extensions_v2",
        importpath = "github.com/matttproud/golang_protobuf_extensions/v2",
        sum = "h1:jWpvCLoY8Z/e3VKvlsiIGKtc+UG6U5vzxaoagmhXfyg=",
        version = "v2.0.0",
    )
    go_repository(
        name = "com_github_microsoft_go_winio",
        importpath = "github.com/Microsoft/go-winio",
        sum = "h1:9/kr64B9VUZrLm5YYwbGtUJnMgqWVOdUAXu6Migciow=",
        version = "v0.6.1",
    )
    go_repository(
        name = "com_github_miekg_dns",
        importpath = "github.com/miekg/dns",
        sum = "h1:5imZaSeoRNvpM9SzWNhEcP9QliKiz20/dA2QabIGVnE=",
        version = "v1.1.56",
    )
    go_repository(
        name = "com_github_mitchellh_copystructure",
        importpath = "github.com/mitchellh/copystructure",
        sum = "h1:vpKXTN4ewci03Vljg/q9QvCGUDttBOGBIa15WveJJGw=",
        version = "v1.2.0",
    )
    go_repository(
        name = "com_github_mitchellh_go_homedir",
        importpath = "github.com/mitchellh/go-homedir",
        sum = "h1:lukF9ziXFxDFPkA1vsr5zpc1XuPDn/wFntq5mG+4E0Y=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_mitchellh_mapstructure",
        importpath = "github.com/mitchellh/mapstructure",
        sum = "h1:jeMsZIYE/09sWLaz43PL7Gy6RuMjD2eJVyuac5Z2hdY=",
        version = "v1.5.0",
    )
    go_repository(
        name = "com_github_mitchellh_reflectwalk",
        importpath = "github.com/mitchellh/reflectwalk",
        sum = "h1:G2LzWKi524PWgd3mLHV8Y5k7s6XUvT0Gef6zxSIeXaQ=",
        version = "v1.0.2",
    )
    go_repository(
        name = "com_github_moby_spdystream",
        importpath = "github.com/moby/spdystream",
        sum = "h1:cjW1zVyyoiM0T7b6UoySUFqzXMoqRckQtXwGPiBhOM8=",
        version = "v0.2.0",
    )
    go_repository(
        name = "com_github_moby_term",
        importpath = "github.com/moby/term",
        sum = "h1:HfkjXDfhgVaN5rmueG8cL8KKeFNecRCXFhaJ2qZ5SKA=",
        version = "v0.0.0-20221205130635-1aeaba878587",
    )
    go_repository(
        name = "com_github_modern_go_concurrent",
        importpath = "github.com/modern-go/concurrent",
        sum = "h1:TRLaZ9cD/w8PVh93nsPXa1VrQ6jlwL5oN8l14QlcNfg=",
        version = "v0.0.0-20180306012644-bacd9c7ef1dd",
    )
    go_repository(
        name = "com_github_modern_go_reflect2",
        importpath = "github.com/modern-go/reflect2",
        sum = "h1:xBagoLtFs94CBntxluKeaWgTMpvLxC4ur3nMaC9Gz0M=",
        version = "v1.0.2",
    )
    go_repository(
        name = "com_github_monochromegane_go_gitignore",
        importpath = "github.com/monochromegane/go-gitignore",
        sum = "h1:n6/2gBQ3RWajuToeY6ZtZTIKv2v7ThUy5KKusIT0yc0=",
        version = "v0.0.0-20200626010858-205db1a8cc00",
    )
    go_repository(
        name = "com_github_morikuni_aec",
        importpath = "github.com/morikuni/aec",
        sum = "h1:nP9CBfwrvYnBRgY6qfDQkygYDmYwOilePFkwzv4dU8A=",
        version = "v1.0.0",
    )
    go_repository(
        name = "com_github_motemen_go_loghttp",
        importpath = "github.com/motemen/go-loghttp",
        sum = "h1:uAI3rnOT1OSSY4PUtI/M1orb3q0ewkovwd3wr8xSno4=",
        version = "v0.0.0-20170804080138-974ac5ceac27",
    )
    go_repository(
        name = "com_github_motemen_go_nuts",
        importpath = "github.com/motemen/go-nuts",
        sum = "h1:lQ+0Zt2gm+w5+9iaBWKdJXC/gMrWjHhNbw9ts/9rSZ4=",
        version = "v0.0.0-20220604134737-2658d0104f31",
    )
    go_repository(
        name = "com_github_munnerz_goautoneg",
        importpath = "github.com/munnerz/goautoneg",
        sum = "h1:C3w9PqII01/Oq1c1nUAm88MOHcQC9l5mIlSMApZMrHA=",
        version = "v0.0.0-20191010083416-a7dc8b61c822",
    )
    go_repository(
        name = "com_github_mwitkow_go_conntrack",
        importpath = "github.com/mwitkow/go-conntrack",
        sum = "h1:KUppIJq7/+SVif2QVs3tOP0zanoHgBEVAwHxUSIzRqU=",
        version = "v0.0.0-20190716064945-2f068394615f",
    )
    go_repository(
        name = "com_github_mxk_go_flowrate",
        importpath = "github.com/mxk/go-flowrate",
        sum = "h1:y5//uYreIhSUg3J1GEMiLbxo1LJaP8RfCpH6pymGZus=",
        version = "v0.0.0-20140419014527-cca7078d478f",
    )
    go_repository(
        name = "com_github_nbio_st",
        importpath = "github.com/nbio/st",
        sum = "h1:W6apQkHrMkS0Muv8G/TipAy/FJl/rCYT0+EuS8+Z0z4=",
        version = "v0.0.0-20140626010706-e9e8d9816f32",
    )
    go_repository(
        name = "com_github_niemeyer_pretty",
        importpath = "github.com/niemeyer/pretty",
        sum = "h1:fD57ERR4JtEqsWbfPhv4DMiApHyliiK5xCTNVSPiaAs=",
        version = "v0.0.0-20200227124842-a10e7caefd8e",
    )
    go_repository(
        name = "com_github_nytimes_gziphandler",
        importpath = "github.com/NYTimes/gziphandler",
        sum = "h1:ZUDjpQae29j0ryrS0u/B8HZfJBtBQHjqw2rQ2cqUQ3I=",
        version = "v1.1.1",
    )
    go_repository(
        name = "com_github_oklog_run",
        importpath = "github.com/oklog/run",
        sum = "h1:GEenZ1cK0+q0+wsJew9qUg/DyD8k3JzYsZAi5gYi2mA=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_oklog_ulid",
        importpath = "github.com/oklog/ulid",
        sum = "h1:EGfNDEx6MqHz8B3uNV6QAib1UR2Lm97sHi3ocA6ESJ4=",
        version = "v1.3.1",
    )
    go_repository(
        name = "com_github_onsi_ginkgo_v2",
        importpath = "github.com/onsi/ginkgo/v2",
        sum = "h1:WgqUCUt/lT6yXoQ8Wef0fsNn5cAuMK7+KT9UFRz2tcU=",
        version = "v2.11.0",
    )
    go_repository(
        name = "com_github_onsi_gomega",
        importpath = "github.com/onsi/gomega",
        sum = "h1:naR28SdDFlqrG6kScpT8VWpu1xWY5nJRCF3XaYyBjhI=",
        version = "v1.27.10",
    )
    go_repository(
        name = "com_github_opencontainers_go_digest",
        importpath = "github.com/opencontainers/go-digest",
        sum = "h1:apOUWs51W5PlhuyGyz9FCeeBIOUDA/6nW8Oi/yOhh5U=",
        version = "v1.0.0",
    )
    go_repository(
        name = "com_github_opencontainers_image_spec",
        importpath = "github.com/opencontainers/image-spec",
        sum = "h1:9yCKha/T5XdGtO0q9Q9a6T5NUCsTn/DrBg0D7ufOcFM=",
        version = "v1.0.2",
    )
    go_repository(
        name = "com_github_ovh_go_ovh",
        importpath = "github.com/ovh/go-ovh",
        sum = "h1:Gs3V823zwTFpzgGLZNI6ILS4rmxZgJwJCz54Er9LwD0=",
        version = "v1.4.3",
    )
    go_repository(
        name = "com_github_pelletier_go_toml",
        importpath = "github.com/pelletier/go-toml",
        sum = "h1:4yBQzkHv+7BHq2PQUZF3Mx0IYxG7LsP222s7Agd3ve8=",
        version = "v1.9.5",
    )
    go_repository(
        name = "com_github_peterbourgon_diskv",
        importpath = "github.com/peterbourgon/diskv",
        sum = "h1:UBdAOUP5p4RWqPBg048CAvpKN+vxiaj6gdUUzhl4XmI=",
        version = "v2.0.1+incompatible",
    )
    go_repository(
        name = "com_github_pkg_browser",
        importpath = "github.com/pkg/browser",
        sum = "h1:KoWmjvw+nsYOo29YJK9vDA65RGE3NrOnUtO7a+RF9HU=",
        version = "v0.0.0-20210911075715-681adbf594b8",
    )
    go_repository(
        name = "com_github_pkg_errors",
        importpath = "github.com/pkg/errors",
        sum = "h1:FEBLx1zS214owpjy7qsBeixbURkuhQAwrK5UwLGTwt4=",
        version = "v0.9.1",
    )
    go_repository(
        name = "com_github_pmezard_go_difflib",
        importpath = "github.com/pmezard/go-difflib",
        sum = "h1:Jamvg5psRIccs7FGNTlIRMkT8wgtp5eCXdBlqhYGL6U=",
        version = "v1.0.1-0.20181226105442-5d4384ee4fb2",
    )
    go_repository(
        name = "com_github_prometheus_alertmanager",
        importpath = "github.com/prometheus/alertmanager",
        sum = "h1:uOMJWfIwJguc3NaM3appWNbbrh6G/OjvaHMk22aBBYc=",
        version = "v0.26.0",
    )
    go_repository(
        name = "com_github_prometheus_client_golang",
        importpath = "github.com/prometheus/client_golang",
        sum = "h1:rl2sfwZMtSthVU752MqfjQozy7blglC+1SOtjMAMh+Q=",
        version = "v1.17.0",
    )
    go_repository(
        name = "com_github_prometheus_client_model",
        importpath = "github.com/prometheus/client_model",
        sum = "h1:VQw1hfvPvk3Uv6Qf29VrPF32JB6rtbgI6cYPYQjL0Qw=",
        version = "v0.5.0",
    )
    go_repository(
        name = "com_github_prometheus_common",
        importpath = "github.com/prometheus/common",
        sum = "h1:2BGz0eBc2hdMDLnO/8n0jeB3oPrt2D08CekT0lneoxM=",
        version = "v0.45.0",
    )
    go_repository(
        name = "com_github_prometheus_common_assets",
        importpath = "github.com/prometheus/common/assets",
        sum = "h1:0P5OrzoHrYBOSM1OigWL3mY8ZvV2N4zIE/5AahrSrfM=",
        version = "v0.2.0",
    )
    go_repository(
        name = "com_github_prometheus_common_sigv4",
        importpath = "github.com/prometheus/common/sigv4",
        sum = "h1:qoVebwtwwEhS85Czm2dSROY5fTo2PAPEVdDeppTwGX4=",
        version = "v0.1.0",
    )
    go_repository(
        name = "com_github_prometheus_exporter_toolkit",
        importpath = "github.com/prometheus/exporter-toolkit",
        sum = "h1:yOAzZTi4M22ZzVxD+fhy1URTuNRj/36uQJJ5S8IPza8=",
        version = "v0.10.0",
    )
    go_repository(
        name = "com_github_prometheus_procfs",
        importpath = "github.com/prometheus/procfs",
        sum = "h1:jluTpSng7V9hY0O2R9DzzJHYb2xULk9VTR1V1R/k6Bo=",
        version = "v0.12.0",
    )
    go_repository(
        name = "com_github_prometheus_prometheus",
        importpath = "github.com/prometheus/prometheus",
        sum = "h1:yrBloImGQ7je4h8M10ujGh4R6oxYQJQKlMuETwNskGk=",
        version = "v0.48.0",
    )
    go_repository(
        name = "com_github_prometheus_statsd_exporter",
        importpath = "github.com/prometheus/statsd_exporter",
        sum = "h1:Qo2D9ZzaQG+id9i5NYNGmbf1aa/KxKbB9aKfMS+Yib0=",
        version = "v0.22.8",
    )
    go_repository(
        name = "com_github_rakyll_embedmd",
        importpath = "github.com/rakyll/embedmd",
        sum = "h1:1jfy6i1g66ijpffgfaF/7pIFYZnSZzvo9P9DFkFmRIM=",
        version = "v0.0.0-20171029212350-c8060a0752a2",
    )
    go_repository(
        name = "com_github_rogpeppe_go_internal",
        importpath = "github.com/rogpeppe/go-internal",
        sum = "h1:TMyTOH3F/DB16zRVcYyreMH6GnZZrwQVAoYjRBZyWFQ=",
        version = "v1.10.0",
    )
    go_repository(
        name = "com_github_russross_blackfriday_v2",
        importpath = "github.com/russross/blackfriday/v2",
        sum = "h1:JIOH55/0cWyOuilr9/qlrm0BSXldqnqwMsf35Ld67mk=",
        version = "v2.1.0",
    )

    # keep
    go_repository(
        name = "com_github_ryanuber_go_license",
        importpath = "github.com/ryanuber/go-license",
        sum = "h1:tRp20LMuPNq4xTO4SLHTxVySYje3m5hLlu5RZLvaY/c=",
        version = "v0.0.0-20180405065157-c69f41c2c8d6",
    )
    go_repository(
        name = "com_github_saintfish_chardet",
        importpath = "github.com/saintfish/chardet",
        sum = "h1:NugYot0LIVPxTvN8n+Kvkn6TrbMyxQiuvKdEwFdR9vI=",
        version = "v0.0.0-20120816061221-3af4cd4741ca",
    )
    go_repository(
        name = "com_github_scaleway_scaleway_sdk_go",
        importpath = "github.com/scaleway/scaleway-sdk-go",
        sum = "h1:yWfiTPwYxB0l5fGMhl/G+liULugVIHD9AU77iNLrURQ=",
        version = "v1.0.0-beta.21",
    )
    go_repository(
        name = "com_github_sergi_go_diff",
        importpath = "github.com/sergi/go-diff",
        sum = "h1:we8PVUC3FE2uYfodKH/nBHMSetSfHDR6scGdBi+erh0=",
        version = "v1.1.0",
    )
    go_repository(
        name = "com_github_shurcool_httpfs",
        importpath = "github.com/shurcooL/httpfs",
        sum = "h1:aqg5Vm5dwtvL+YgDpBcK1ITf3o96N/K7/wsRXQnUTEs=",
        version = "v0.0.0-20230704072500-f1e31cf0ba5c",
    )
    go_repository(
        name = "com_github_sirupsen_logrus",
        importpath = "github.com/sirupsen/logrus",
        sum = "h1:trlNQbNUG3OdDrDil03MCb1H2o9nJ1x4/5LYw7byDE0=",
        version = "v1.9.0",
    )
    go_repository(
        name = "com_github_soheilhy_cmux",
        importpath = "github.com/soheilhy/cmux",
        sum = "h1:jjzc5WVemNEDTLwv9tlmemhC73tI08BNOIGwBOo10Js=",
        version = "v0.1.5",
    )
    go_repository(
        name = "com_github_spf13_cobra",
        importpath = "github.com/spf13/cobra",
        sum = "h1:7aJaZx1B85qltLMc546zn58BxxfZdR/W22ej9CFoEf0=",
        version = "v1.8.0",
    )
    go_repository(
        name = "com_github_spf13_pflag",
        importpath = "github.com/spf13/pflag",
        sum = "h1:iy+VFUOCP1a+8yFto/drg2CJ5u0yRoB7fZw3DKv/JXA=",
        version = "v1.0.5",
    )
    go_repository(
        name = "com_github_stoewer_go_strcase",
        importpath = "github.com/stoewer/go-strcase",
        sum = "h1:Z2iHWqGXH00XYgqDmNgQbIBxf3wrNq0F3feEy0ainaU=",
        version = "v1.2.0",
    )
    go_repository(
        name = "com_github_stretchr_objx",
        importpath = "github.com/stretchr/objx",
        sum = "h1:1zr/of2m5FGMsad5YfcqgdqdWrIhu+EBEJRhR1U7z/c=",
        version = "v0.5.0",
    )
    go_repository(
        name = "com_github_stretchr_testify",
        importpath = "github.com/stretchr/testify",
        sum = "h1:CcVxjf3Q8PM0mHUKJCdn+eZZtm5yQwehR5yeSVQQcUk=",
        version = "v1.8.4",
    )
    go_repository(
        name = "com_github_stvp_go_udp_testing",
        importpath = "github.com/stvp/go-udp-testing",
        sum = "h1:LUsDduamlucuNnWcaTbXQ6aLILFcLXADpOzeEH3U+OI=",
        version = "v0.0.0-20201019212854-469649b16807",
    )
    go_repository(
        name = "com_github_tmc_grpc_websocket_proxy",
        importpath = "github.com/tmc/grpc-websocket-proxy",
        sum = "h1:6fotK7otjonDflCTK0BCfls4SPy3NcCVb5dqqmbRknE=",
        version = "v0.0.0-20220101234140-673ab2c3ae75",
    )
    go_repository(
        name = "com_github_vultr_govultr_v2",
        importpath = "github.com/vultr/govultr/v2",
        sum = "h1:gej/rwr91Puc/tgh+j33p/BLR16UrIPnSr+AIwYWZQs=",
        version = "v2.17.2",
    )
    go_repository(
        name = "com_github_xhit_go_str2duration_v2",
        importpath = "github.com/xhit/go-str2duration/v2",
        sum = "h1:lxklc02Drh6ynqX+DdPyp5pCKLUQpRT8bp8Ydu2Bstc=",
        version = "v2.1.0",
    )
    go_repository(
        name = "com_github_xiang90_probing",
        importpath = "github.com/xiang90/probing",
        sum = "h1:eY9dn8+vbi4tKz5Qo6v2eYzo7kUS51QINcR5jNpbZS8=",
        version = "v0.0.0-20190116061207-43a291ad63a2",
    )
    go_repository(
        name = "com_github_xlab_treeprint",
        importpath = "github.com/xlab/treeprint",
        sum = "h1:HzHnuAF1plUN2zGlAFHbSQP2qJ0ZAD3XF5XD7OesXRQ=",
        version = "v1.2.0",
    )
    go_repository(
        name = "com_github_yuin_goldmark",
        importpath = "github.com/yuin/goldmark",
        sum = "h1:fVcFKWvrslecOb/tg+Cc05dkeYx540o0FuFt3nUVDoE=",
        version = "v1.4.13",
    )
    go_repository(
        name = "com_google_cloud_go",
        importpath = "cloud.google.com/go",
        sum = "h1:LXy9GEO+timppncPIAZoOj3l58LIU9k+kn48AN7IO3Y=",
        version = "v0.110.10",
    )
    go_repository(
        name = "com_google_cloud_go_accessapproval",
        importpath = "cloud.google.com/go/accessapproval",
        sum = "h1:ZvLvJ952zK8pFHINjpMBY5k7LTAp/6pBf50RDMRgBUI=",
        version = "v1.7.4",
    )
    go_repository(
        name = "com_google_cloud_go_accesscontextmanager",
        importpath = "cloud.google.com/go/accesscontextmanager",
        sum = "h1:Yo4g2XrBETBCqyWIibN3NHNPQKUfQqti0lI+70rubeE=",
        version = "v1.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_aiplatform",
        importpath = "cloud.google.com/go/aiplatform",
        sum = "h1:wH7OYl9Vq/5tupok0BPTFY9xaTLb0GxkReHtB5PF7cI=",
        version = "v1.54.0",
    )
    go_repository(
        name = "com_google_cloud_go_analytics",
        importpath = "cloud.google.com/go/analytics",
        sum = "h1:fnV7B8lqyEYxCU0LKk+vUL7mTlqRAq4uFlIthIdr/iA=",
        version = "v0.21.6",
    )
    go_repository(
        name = "com_google_cloud_go_apigateway",
        importpath = "cloud.google.com/go/apigateway",
        sum = "h1:VVIxCtVerchHienSlaGzV6XJGtEM9828Erzyr3miUGs=",
        version = "v1.6.4",
    )
    go_repository(
        name = "com_google_cloud_go_apigeeconnect",
        importpath = "cloud.google.com/go/apigeeconnect",
        sum = "h1:jSoGITWKgAj/ssVogNE9SdsTqcXnryPzsulENSRlusI=",
        version = "v1.6.4",
    )
    go_repository(
        name = "com_google_cloud_go_apigeeregistry",
        importpath = "cloud.google.com/go/apigeeregistry",
        sum = "h1:DSaD1iiqvELag+lV4VnnqUUFd8GXELu01tKVdWZrviE=",
        version = "v0.8.2",
    )
    go_repository(
        name = "com_google_cloud_go_appengine",
        importpath = "cloud.google.com/go/appengine",
        sum = "h1:Qub3fqR7iA1daJWdzjp/Q0Jz0fUG0JbMc7Ui4E9IX/E=",
        version = "v1.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_area120",
        importpath = "cloud.google.com/go/area120",
        sum = "h1:YnSO8m02pOIo6AEOgiOoUDVbw4pf+bg2KLHi4rky320=",
        version = "v0.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_artifactregistry",
        importpath = "cloud.google.com/go/artifactregistry",
        sum = "h1:/hQaadYytMdA5zBh+RciIrXZQBWK4vN7EUsrQHG+/t8=",
        version = "v1.14.6",
    )
    go_repository(
        name = "com_google_cloud_go_asset",
        importpath = "cloud.google.com/go/asset",
        sum = "h1:uI8Bdm81s0esVWbWrTHcjFDFKNOa9aB7rI1vud1hO84=",
        version = "v1.15.3",
    )
    go_repository(
        name = "com_google_cloud_go_assuredworkloads",
        importpath = "cloud.google.com/go/assuredworkloads",
        sum = "h1:FsLSkmYYeNuzDm8L4YPfLWV+lQaUrJmH5OuD37t1k20=",
        version = "v1.11.4",
    )
    go_repository(
        name = "com_google_cloud_go_automl",
        importpath = "cloud.google.com/go/automl",
        sum = "h1:i9tOKXX+1gE7+rHpWKjiuPfGBVIYoWvLNIGpWgPtF58=",
        version = "v1.13.4",
    )
    go_repository(
        name = "com_google_cloud_go_baremetalsolution",
        importpath = "cloud.google.com/go/baremetalsolution",
        sum = "h1:oQiFYYCe0vwp7J8ZmF6siVKEumWtiPFJMJcGuyDVRUk=",
        version = "v1.2.3",
    )
    go_repository(
        name = "com_google_cloud_go_batch",
        importpath = "cloud.google.com/go/batch",
        sum = "h1:mPiIH20a5NU02rucbAmLeO4sLPO9hrTK0BLjdHyW8xw=",
        version = "v1.6.3",
    )
    go_repository(
        name = "com_google_cloud_go_beyondcorp",
        importpath = "cloud.google.com/go/beyondcorp",
        sum = "h1:VXf9SnrnSmj2BF2cHkoTHvOUp8gjsz1KJFOMW7czdsY=",
        version = "v1.0.3",
    )
    go_repository(
        name = "com_google_cloud_go_bigquery",
        importpath = "cloud.google.com/go/bigquery",
        sum = "h1:FiULdbbzUxWD0Y4ZGPSVCDLvqRSyCIO6zKV7E2nf5uA=",
        version = "v1.57.1",
    )
    go_repository(
        name = "com_google_cloud_go_billing",
        importpath = "cloud.google.com/go/billing",
        sum = "h1:77/4kCqzH6Ou5CCDzNmqmboE+WvbwFBJmw1QZQz19AI=",
        version = "v1.17.4",
    )
    go_repository(
        name = "com_google_cloud_go_binaryauthorization",
        importpath = "cloud.google.com/go/binaryauthorization",
        sum = "h1:3R6WYn1JKIaVicBmo18jXubu7xh4mMkmbIgsTXk0cBA=",
        version = "v1.7.3",
    )
    go_repository(
        name = "com_google_cloud_go_certificatemanager",
        importpath = "cloud.google.com/go/certificatemanager",
        sum = "h1:5YMQ3Q+dqGpwUZ9X5sipsOQ1fLPsxod9HNq0+nrqc6I=",
        version = "v1.7.4",
    )
    go_repository(
        name = "com_google_cloud_go_channel",
        importpath = "cloud.google.com/go/channel",
        sum = "h1:Rd4+fBrjiN6tZ4TR8R/38elkyEkz6oogGDr7jDyjmMY=",
        version = "v1.17.3",
    )
    go_repository(
        name = "com_google_cloud_go_cloudbuild",
        importpath = "cloud.google.com/go/cloudbuild",
        sum = "h1:9IHfEMWdCklJ1cwouoiQrnxmP0q3pH7JUt8Hqx4Qbck=",
        version = "v1.15.0",
    )
    go_repository(
        name = "com_google_cloud_go_clouddms",
        importpath = "cloud.google.com/go/clouddms",
        sum = "h1:xe/wJKz55VO1+L891a1EG9lVUgfHr9Ju/I3xh1nwF84=",
        version = "v1.7.3",
    )
    go_repository(
        name = "com_google_cloud_go_cloudtasks",
        importpath = "cloud.google.com/go/cloudtasks",
        sum = "h1:5xXuFfAjg0Z5Wb81j2GAbB3e0bwroCeSF+5jBn/L650=",
        version = "v1.12.4",
    )
    go_repository(
        name = "com_google_cloud_go_compute",
        importpath = "cloud.google.com/go/compute",
        sum = "h1:6sVlXXBmbd7jNX0Ipq0trII3e4n1/MsADLK6a+aiVlk=",
        version = "v1.23.3",
    )
    go_repository(
        name = "com_google_cloud_go_compute_metadata",
        importpath = "cloud.google.com/go/compute/metadata",
        sum = "h1:mg4jlk7mCAj6xXp9UJ4fjI9VUI5rubuGBW5aJ7UnBMY=",
        version = "v0.2.3",
    )
    go_repository(
        name = "com_google_cloud_go_contactcenterinsights",
        importpath = "cloud.google.com/go/contactcenterinsights",
        sum = "h1:wP41IUA4ucMVooj/TP53jd7vbNjWrDkAPOeulVJGT5U=",
        version = "v1.12.0",
    )
    go_repository(
        name = "com_google_cloud_go_container",
        importpath = "cloud.google.com/go/container",
        sum = "h1:/o82CFWXIYnT9p/07SnRgybqL3Pmmu86jYIlzlJVUBY=",
        version = "v1.28.0",
    )
    go_repository(
        name = "com_google_cloud_go_containeranalysis",
        importpath = "cloud.google.com/go/containeranalysis",
        sum = "h1:5rhYLX+3a01drpREqBZVXR9YmWH45RnML++8NsCtuD8=",
        version = "v0.11.3",
    )
    go_repository(
        name = "com_google_cloud_go_datacatalog",
        importpath = "cloud.google.com/go/datacatalog",
        sum = "h1:rbYNmHwvAOOwnW2FPXYkaK3Mf1MmGqRzK0mMiIEyLdo=",
        version = "v1.19.0",
    )
    go_repository(
        name = "com_google_cloud_go_dataflow",
        importpath = "cloud.google.com/go/dataflow",
        sum = "h1:7VmCNWcPJBS/srN2QnStTB6nu4Eb5TMcpkmtaPVhRt4=",
        version = "v0.9.4",
    )
    go_repository(
        name = "com_google_cloud_go_dataform",
        importpath = "cloud.google.com/go/dataform",
        sum = "h1:jV+EsDamGX6cE127+QAcCR/lergVeeZdEQ6DdrxW3sQ=",
        version = "v0.9.1",
    )
    go_repository(
        name = "com_google_cloud_go_datafusion",
        importpath = "cloud.google.com/go/datafusion",
        sum = "h1:Q90alBEYlMi66zL5gMSGQHfbZLB55mOAg03DhwTTfsk=",
        version = "v1.7.4",
    )
    go_repository(
        name = "com_google_cloud_go_datalabeling",
        importpath = "cloud.google.com/go/datalabeling",
        sum = "h1:zrq4uMmunf2KFDl/7dS6iCDBBAxBnKVDyw6+ajz3yu0=",
        version = "v0.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_dataplex",
        importpath = "cloud.google.com/go/dataplex",
        sum = "h1:AfFFR15Ifh4U+Me1IBztrSd5CrasTODzy3x8KtDyHdc=",
        version = "v1.11.2",
    )
    go_repository(
        name = "com_google_cloud_go_dataproc_v2",
        importpath = "cloud.google.com/go/dataproc/v2",
        sum = "h1:tTVP9tTxmc8fixxOd/8s6Q6Pz/+yzn7r7XdZHretQH0=",
        version = "v2.3.0",
    )
    go_repository(
        name = "com_google_cloud_go_dataqna",
        importpath = "cloud.google.com/go/dataqna",
        sum = "h1:NJnu1kAPamZDs/if3bJ3+Wb6tjADHKL83NUWsaIp2zg=",
        version = "v0.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_datastore",
        importpath = "cloud.google.com/go/datastore",
        sum = "h1:0P9WcsQeTWjuD1H14JIY7XQscIPQ4Laje8ti96IC5vg=",
        version = "v1.15.0",
    )
    go_repository(
        name = "com_google_cloud_go_datastream",
        importpath = "cloud.google.com/go/datastream",
        sum = "h1:Z2sKPIB7bT2kMW5Uhxy44ZgdJzxzE5uKjavoW+EuHEE=",
        version = "v1.10.3",
    )
    go_repository(
        name = "com_google_cloud_go_deploy",
        importpath = "cloud.google.com/go/deploy",
        sum = "h1:ZdmYzRMTGkVyP1nXEUat9FpbJGJemDcNcx82RSSOElc=",
        version = "v1.15.0",
    )
    go_repository(
        name = "com_google_cloud_go_dialogflow",
        importpath = "cloud.google.com/go/dialogflow",
        sum = "h1:cK/f88KX+YVR4tLH4clMQlvrLWD2qmKJQziusjGPjmc=",
        version = "v1.44.3",
    )
    go_repository(
        name = "com_google_cloud_go_dlp",
        importpath = "cloud.google.com/go/dlp",
        sum = "h1:OFlXedmPP/5//X1hBEeq3D9kUVm9fb6ywYANlpv/EsQ=",
        version = "v1.11.1",
    )
    go_repository(
        name = "com_google_cloud_go_documentai",
        importpath = "cloud.google.com/go/documentai",
        sum = "h1:KAlzT+q8qvRxAmhsJUvLtfFHH0PNvz3M79H6CgVBKL8=",
        version = "v1.23.5",
    )
    go_repository(
        name = "com_google_cloud_go_domains",
        importpath = "cloud.google.com/go/domains",
        sum = "h1:ua4GvsDztZ5F3xqjeLKVRDeOvJshf5QFgWGg1CKti3A=",
        version = "v0.9.4",
    )
    go_repository(
        name = "com_google_cloud_go_edgecontainer",
        importpath = "cloud.google.com/go/edgecontainer",
        sum = "h1:Szy3Q/N6bqgQGyxqjI+6xJZbmvPvnFHp3UZr95DKcQ0=",
        version = "v1.1.4",
    )
    go_repository(
        name = "com_google_cloud_go_errorreporting",
        importpath = "cloud.google.com/go/errorreporting",
        sum = "h1:kj1XEWMu8P0qlLhm3FwcaFsUvXChV/OraZwA70trRR0=",
        version = "v0.3.0",
    )
    go_repository(
        name = "com_google_cloud_go_essentialcontacts",
        importpath = "cloud.google.com/go/essentialcontacts",
        sum = "h1:S2if6wkjR4JCEAfDtIiYtD+sTz/oXjh2NUG4cgT1y/Q=",
        version = "v1.6.5",
    )
    go_repository(
        name = "com_google_cloud_go_eventarc",
        importpath = "cloud.google.com/go/eventarc",
        sum = "h1:+pFmO4eu4dOVipSaFBLkmqrRYG94Xl/TQZFOeohkuqU=",
        version = "v1.13.3",
    )
    go_repository(
        name = "com_google_cloud_go_filestore",
        importpath = "cloud.google.com/go/filestore",
        sum = "h1:/+wUEGwk3x3Kxomi2cP5dsR8+SIXxo7M0THDjreFSYo=",
        version = "v1.8.0",
    )
    go_repository(
        name = "com_google_cloud_go_firestore",
        importpath = "cloud.google.com/go/firestore",
        sum = "h1:8aLcKnMPoldYU3YHgu4t2exrKhLQkqaXAGqT0ljrFVw=",
        version = "v1.14.0",
    )
    go_repository(
        name = "com_google_cloud_go_functions",
        importpath = "cloud.google.com/go/functions",
        sum = "h1:ZjdiV3MyumRM6++1Ixu6N0VV9LAGlCX4AhW6Yjr1t+U=",
        version = "v1.15.4",
    )
    go_repository(
        name = "com_google_cloud_go_gkebackup",
        importpath = "cloud.google.com/go/gkebackup",
        sum = "h1:KhnOrr9A1tXYIYeXKqCKbCI8TL2ZNGiD3dm+d7BDUBg=",
        version = "v1.3.4",
    )
    go_repository(
        name = "com_google_cloud_go_gkeconnect",
        importpath = "cloud.google.com/go/gkeconnect",
        sum = "h1:1JLpZl31YhQDQeJ98tK6QiwTpgHFYRJwpntggpQQWis=",
        version = "v0.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_gkehub",
        importpath = "cloud.google.com/go/gkehub",
        sum = "h1:J5tYUtb3r0cl2mM7+YHvV32eL+uZQ7lONyUZnPikCEo=",
        version = "v0.14.4",
    )
    go_repository(
        name = "com_google_cloud_go_gkemulticloud",
        importpath = "cloud.google.com/go/gkemulticloud",
        sum = "h1:NmJsNX9uQ2CT78957xnjXZb26TDIMvv+d5W2vVUt0Pg=",
        version = "v1.0.3",
    )
    go_repository(
        name = "com_google_cloud_go_gsuiteaddons",
        importpath = "cloud.google.com/go/gsuiteaddons",
        sum = "h1:uuw2Xd37yHftViSI8J2hUcCS8S7SH3ZWH09sUDLW30Q=",
        version = "v1.6.4",
    )
    go_repository(
        name = "com_google_cloud_go_iam",
        importpath = "cloud.google.com/go/iam",
        sum = "h1:1jTsCu4bcsNsE4iiqNT5SHwrDRCfRmIaaaVFhRveTJI=",
        version = "v1.1.5",
    )
    go_repository(
        name = "com_google_cloud_go_iap",
        importpath = "cloud.google.com/go/iap",
        sum = "h1:M4vDbQ4TLXdaljXVZSwW7XtxpwXUUarY2lIs66m0aCM=",
        version = "v1.9.3",
    )
    go_repository(
        name = "com_google_cloud_go_ids",
        importpath = "cloud.google.com/go/ids",
        sum = "h1:VuFqv2ctf/A7AyKlNxVvlHTzjrEvumWaZflUzBPz/M4=",
        version = "v1.4.4",
    )
    go_repository(
        name = "com_google_cloud_go_iot",
        importpath = "cloud.google.com/go/iot",
        sum = "h1:m1WljtkZnvLTIRYW1YTOv5A6H1yKgLHR6nU7O8yf27w=",
        version = "v1.7.4",
    )
    go_repository(
        name = "com_google_cloud_go_kms",
        importpath = "cloud.google.com/go/kms",
        sum = "h1:pj1sRfut2eRbD9pFRjNnPNg/CzJPuQAzUujMIM1vVeM=",
        version = "v1.15.5",
    )
    go_repository(
        name = "com_google_cloud_go_language",
        importpath = "cloud.google.com/go/language",
        sum = "h1:zg9uq2yS9PGIOdc0Kz/l+zMtOlxKWonZjjo5w5YPG2A=",
        version = "v1.12.2",
    )
    go_repository(
        name = "com_google_cloud_go_lifesciences",
        importpath = "cloud.google.com/go/lifesciences",
        sum = "h1:rZEI/UxcxVKEzyoRS/kdJ1VoolNItRWjNN0Uk9tfexg=",
        version = "v0.9.4",
    )
    go_repository(
        name = "com_google_cloud_go_logging",
        importpath = "cloud.google.com/go/logging",
        sum = "h1:26skQWPeYhvIasWKm48+Eq7oUqdcdbwsCVwz5Ys0FvU=",
        version = "v1.8.1",
    )
    go_repository(
        name = "com_google_cloud_go_longrunning",
        importpath = "cloud.google.com/go/longrunning",
        sum = "h1:w8xEcbZodnA2BbW6sVirkkoC+1gP8wS57EUUgGS0GVg=",
        version = "v0.5.4",
    )
    go_repository(
        name = "com_google_cloud_go_managedidentities",
        importpath = "cloud.google.com/go/managedidentities",
        sum = "h1:SF/u1IJduMqQQdJA4MDyivlIQ4SrV5qAawkr/ZEREkY=",
        version = "v1.6.4",
    )
    go_repository(
        name = "com_google_cloud_go_maps",
        importpath = "cloud.google.com/go/maps",
        sum = "h1:2+eMp/1MvMPp5qrSOd3vtnLKa/pylt+krVRqET3jWsM=",
        version = "v1.6.1",
    )
    go_repository(
        name = "com_google_cloud_go_mediatranslation",
        importpath = "cloud.google.com/go/mediatranslation",
        sum = "h1:VRCQfZB4s6jN0CSy7+cO3m4ewNwgVnaePanVCQh/9Z4=",
        version = "v0.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_memcache",
        importpath = "cloud.google.com/go/memcache",
        sum = "h1:cdex/ayDd294XBj2cGeMe6Y+H1JvhN8y78B9UW7pxuQ=",
        version = "v1.10.4",
    )
    go_repository(
        name = "com_google_cloud_go_metastore",
        importpath = "cloud.google.com/go/metastore",
        sum = "h1:94l/Yxg9oBZjin2bzI79oK05feYefieDq0o5fjLSkC8=",
        version = "v1.13.3",
    )
    go_repository(
        name = "com_google_cloud_go_monitoring",
        importpath = "cloud.google.com/go/monitoring",
        sum = "h1:mf2SN9qSoBtIgiMA4R/y4VADPWZA7VCNJA079qLaZQ8=",
        version = "v1.16.3",
    )
    go_repository(
        name = "com_google_cloud_go_networkconnectivity",
        importpath = "cloud.google.com/go/networkconnectivity",
        sum = "h1:e9lUkCe2BexsqsUc2bjV8+gFBpQa54J+/F3qKVtW+wA=",
        version = "v1.14.3",
    )
    go_repository(
        name = "com_google_cloud_go_networkmanagement",
        importpath = "cloud.google.com/go/networkmanagement",
        sum = "h1:HsQk4FNKJUX04k3OI6gUsoveiHMGvDRqlaFM2xGyvqU=",
        version = "v1.9.3",
    )
    go_repository(
        name = "com_google_cloud_go_networksecurity",
        importpath = "cloud.google.com/go/networksecurity",
        sum = "h1:947tNIPnj1bMGTIEBo3fc4QrrFKS5hh0bFVsHmFm4Vo=",
        version = "v0.9.4",
    )
    go_repository(
        name = "com_google_cloud_go_notebooks",
        importpath = "cloud.google.com/go/notebooks",
        sum = "h1:eTOTfNL1yM6L/PCtquJwjWg7ZZGR0URFaFgbs8kllbM=",
        version = "v1.11.2",
    )
    go_repository(
        name = "com_google_cloud_go_optimization",
        importpath = "cloud.google.com/go/optimization",
        sum = "h1:iFsoexcp13cGT3k/Hv8PA5aK+FP7FnbhwDO9llnruas=",
        version = "v1.6.2",
    )
    go_repository(
        name = "com_google_cloud_go_orchestration",
        importpath = "cloud.google.com/go/orchestration",
        sum = "h1:kgwZ2f6qMMYIVBtUGGoU8yjYWwMTHDanLwM/CQCFaoQ=",
        version = "v1.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_orgpolicy",
        importpath = "cloud.google.com/go/orgpolicy",
        sum = "h1:RWuXQDr9GDYhjmrredQJC7aY7cbyqP9ZuLbq5GJGves=",
        version = "v1.11.4",
    )
    go_repository(
        name = "com_google_cloud_go_osconfig",
        importpath = "cloud.google.com/go/osconfig",
        sum = "h1:OrRCIYEAbrbXdhm13/JINn9pQchvTTIzgmOCA7uJw8I=",
        version = "v1.12.4",
    )
    go_repository(
        name = "com_google_cloud_go_oslogin",
        importpath = "cloud.google.com/go/oslogin",
        sum = "h1:NP/KgsD9+0r9hmHC5wKye0vJXVwdciv219DtYKYjgqE=",
        version = "v1.12.2",
    )
    go_repository(
        name = "com_google_cloud_go_phishingprotection",
        importpath = "cloud.google.com/go/phishingprotection",
        sum = "h1:sPLUQkHq6b4AL0czSJZ0jd6vL55GSTHz2B3Md+TCZI0=",
        version = "v0.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_policytroubleshooter",
        importpath = "cloud.google.com/go/policytroubleshooter",
        sum = "h1:sq+ScLP83d7GJy9+wpwYJVnY+q6xNTXwOdRIuYjvHT4=",
        version = "v1.10.2",
    )
    go_repository(
        name = "com_google_cloud_go_privatecatalog",
        importpath = "cloud.google.com/go/privatecatalog",
        sum = "h1:Vo10IpWKbNvc/z/QZPVXgCiwfjpWoZ/wbgful4Uh/4E=",
        version = "v0.9.4",
    )
    go_repository(
        name = "com_google_cloud_go_pubsub",
        importpath = "cloud.google.com/go/pubsub",
        sum = "h1:6SPCPvWav64tj0sVX/+npCBKhUi/UjJehy9op/V3p2g=",
        version = "v1.33.0",
    )
    go_repository(
        name = "com_google_cloud_go_pubsublite",
        importpath = "cloud.google.com/go/pubsublite",
        sum = "h1:pX+idpWMIH30/K7c0epN6V703xpIcMXWRjKJsz0tYGY=",
        version = "v1.8.1",
    )
    go_repository(
        name = "com_google_cloud_go_recaptchaenterprise_v2",
        importpath = "cloud.google.com/go/recaptchaenterprise/v2",
        sum = "h1:KOlLHLv3h3HwcZAkx91ubM3Oztz3JtT3ZacAJhWDorQ=",
        version = "v2.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_recommendationengine",
        importpath = "cloud.google.com/go/recommendationengine",
        sum = "h1:JRiwe4hvu3auuh2hujiTc2qNgPPfVp+Q8KOpsXlEzKQ=",
        version = "v0.8.4",
    )
    go_repository(
        name = "com_google_cloud_go_recommender",
        importpath = "cloud.google.com/go/recommender",
        sum = "h1:VndmgyS/J3+izR8V8BHa7HV/uun8//ivQ3k5eVKKyyM=",
        version = "v1.11.3",
    )
    go_repository(
        name = "com_google_cloud_go_redis",
        importpath = "cloud.google.com/go/redis",
        sum = "h1:J9cEHxG9YLmA9o4jTSvWt/RuVEn6MTrPlYSCRHujxDQ=",
        version = "v1.14.1",
    )
    go_repository(
        name = "com_google_cloud_go_resourcemanager",
        importpath = "cloud.google.com/go/resourcemanager",
        sum = "h1:JwZ7Ggle54XQ/FVYSBrMLOQIKoIT/uer8mmNvNLK51k=",
        version = "v1.9.4",
    )
    go_repository(
        name = "com_google_cloud_go_resourcesettings",
        importpath = "cloud.google.com/go/resourcesettings",
        sum = "h1:yTIL2CsZswmMfFyx2Ic77oLVzfBFoWBYgpkgiSPnC4Y=",
        version = "v1.6.4",
    )
    go_repository(
        name = "com_google_cloud_go_retail",
        importpath = "cloud.google.com/go/retail",
        sum = "h1:geqdX1FNqqL2p0ADXjPpw8lq986iv5GrVcieTYafuJQ=",
        version = "v1.14.4",
    )
    go_repository(
        name = "com_google_cloud_go_run",
        importpath = "cloud.google.com/go/run",
        sum = "h1:qdfZteAm+vgzN1iXzILo3nJFQbzziudkJrvd9wCf3FQ=",
        version = "v1.3.3",
    )
    go_repository(
        name = "com_google_cloud_go_scheduler",
        importpath = "cloud.google.com/go/scheduler",
        sum = "h1:eMEettHlFhG5pXsoHouIM5nRT+k+zU4+GUvRtnxhuVI=",
        version = "v1.10.5",
    )
    go_repository(
        name = "com_google_cloud_go_secretmanager",
        importpath = "cloud.google.com/go/secretmanager",
        sum = "h1:krnX9qpG2kR2fJ+u+uNyNo+ACVhplIAS4Pu7u+4gd+k=",
        version = "v1.11.4",
    )
    go_repository(
        name = "com_google_cloud_go_security",
        importpath = "cloud.google.com/go/security",
        sum = "h1:sdnh4Islb1ljaNhpIXlIPgb3eYj70QWgPVDKOUYvzJc=",
        version = "v1.15.4",
    )
    go_repository(
        name = "com_google_cloud_go_securitycenter",
        importpath = "cloud.google.com/go/securitycenter",
        sum = "h1:qCEyXoJoxNKKA1bDywBjjqCB7ODXazzHnVWnG5Uqd1M=",
        version = "v1.24.2",
    )
    go_repository(
        name = "com_google_cloud_go_servicedirectory",
        importpath = "cloud.google.com/go/servicedirectory",
        sum = "h1:5niCMfkw+jifmFtbBrtRedbXkJm3fubSR/KHbxSJZVM=",
        version = "v1.11.3",
    )
    go_repository(
        name = "com_google_cloud_go_shell",
        importpath = "cloud.google.com/go/shell",
        sum = "h1:nurhlJcSVFZneoRZgkBEHumTYf/kFJptCK2eBUq/88M=",
        version = "v1.7.4",
    )
    go_repository(
        name = "com_google_cloud_go_spanner",
        importpath = "cloud.google.com/go/spanner",
        sum = "h1:/NzWQJ1MEhdRcffiutRKbW/AIGVKhcTeivWTDjEyCCo=",
        version = "v1.53.0",
    )
    go_repository(
        name = "com_google_cloud_go_speech",
        importpath = "cloud.google.com/go/speech",
        sum = "h1:qkxNao58oF8ghAHE1Eghen7XepawYEN5zuZXYWaUTA4=",
        version = "v1.21.0",
    )
    go_repository(
        name = "com_google_cloud_go_storage",
        importpath = "cloud.google.com/go/storage",
        sum = "h1:uOdMxAs8HExqBlnLtnQyP0YkvbiDpdGShGKtx6U/oNM=",
        version = "v1.30.1",
    )
    go_repository(
        name = "com_google_cloud_go_storagetransfer",
        importpath = "cloud.google.com/go/storagetransfer",
        sum = "h1:YM1dnj5gLjfL6aDldO2s4GeU8JoAvH1xyIwXre63KmI=",
        version = "v1.10.3",
    )
    go_repository(
        name = "com_google_cloud_go_talent",
        importpath = "cloud.google.com/go/talent",
        sum = "h1:LnRJhhYkODDBoTwf6BeYkiJHFw9k+1mAFNyArwZUZAs=",
        version = "v1.6.5",
    )
    go_repository(
        name = "com_google_cloud_go_texttospeech",
        importpath = "cloud.google.com/go/texttospeech",
        sum = "h1:ahrzTgr7uAbvebuhkBAAVU6kRwVD0HWsmDsvMhtad5Q=",
        version = "v1.7.4",
    )
    go_repository(
        name = "com_google_cloud_go_tpu",
        importpath = "cloud.google.com/go/tpu",
        sum = "h1:XIEH5c0WeYGaVy9H+UueiTaf3NI6XNdB4/v6TFQJxtE=",
        version = "v1.6.4",
    )
    go_repository(
        name = "com_google_cloud_go_trace",
        importpath = "cloud.google.com/go/trace",
        sum = "h1:2qOAuAzNezwW3QN+t41BtkDJOG42HywL73q8x/f6fnM=",
        version = "v1.10.4",
    )
    go_repository(
        name = "com_google_cloud_go_translate",
        importpath = "cloud.google.com/go/translate",
        sum = "h1:t5WXTqlrk8VVJu/i3WrYQACjzYJiff5szARHiyqqPzI=",
        version = "v1.9.3",
    )
    go_repository(
        name = "com_google_cloud_go_video",
        importpath = "cloud.google.com/go/video",
        sum = "h1:Xrpbm2S9UFQ1pZEeJt9Vqm5t2T/z9y/M3rNXhFoo8Is=",
        version = "v1.20.3",
    )
    go_repository(
        name = "com_google_cloud_go_videointelligence",
        importpath = "cloud.google.com/go/videointelligence",
        sum = "h1:YS4j7lY0zxYyneTFXjBJUj2r4CFe/UoIi/PJG0Zt/Rg=",
        version = "v1.11.4",
    )
    go_repository(
        name = "com_google_cloud_go_vision_v2",
        importpath = "cloud.google.com/go/vision/v2",
        sum = "h1:T/ujUghvEaTb+YnFY/jiYwVAkMbIC8EieK0CJo6B4vg=",
        version = "v2.7.5",
    )
    go_repository(
        name = "com_google_cloud_go_vmmigration",
        importpath = "cloud.google.com/go/vmmigration",
        sum = "h1:qPNdab4aGgtaRX+51jCOtJxlJp6P26qua4o1xxUDjpc=",
        version = "v1.7.4",
    )
    go_repository(
        name = "com_google_cloud_go_vmwareengine",
        importpath = "cloud.google.com/go/vmwareengine",
        sum = "h1:WY526PqM6QNmFHSqe2sRfK6gRpzWjmL98UFkql2+JDM=",
        version = "v1.0.3",
    )
    go_repository(
        name = "com_google_cloud_go_vpcaccess",
        importpath = "cloud.google.com/go/vpcaccess",
        sum = "h1:zbs3V+9ux45KYq8lxxn/wgXole6SlBHHKKyZhNJoS+8=",
        version = "v1.7.4",
    )
    go_repository(
        name = "com_google_cloud_go_webrisk",
        importpath = "cloud.google.com/go/webrisk",
        sum = "h1:iceR3k0BCRZgf2D/NiKviVMFfuNC9LmeNLtxUFRB/wI=",
        version = "v1.9.4",
    )
    go_repository(
        name = "com_google_cloud_go_websecurityscanner",
        importpath = "cloud.google.com/go/websecurityscanner",
        sum = "h1:5Gp7h5j7jywxLUp6NTpjNPkgZb3ngl0tUSw6ICWvtJQ=",
        version = "v1.6.4",
    )
    go_repository(
        name = "com_google_cloud_go_workflows",
        importpath = "cloud.google.com/go/workflows",
        sum = "h1:qocsqETmLAl34mSa01hKZjcqAvt699gaoFbooGGMvaM=",
        version = "v1.12.3",
    )
    go_repository(
        name = "com_shuralyov_dmitri_gpu_mtl",
        importpath = "dmitri.shuralyov.com/gpu/mtl",
        sum = "h1:VpgP7xuJadIUuKccphEpTJnWhS2jkQyMt6Y7pJCD7fY=",
        version = "v0.0.0-20190408044501-666a987793e9",
    )
    go_repository(
        name = "in_gopkg_alecthomas_kingpin_v2",
        importpath = "gopkg.in/alecthomas/kingpin.v2",
        sum = "h1:jMFz6MfLP0/4fUyZle81rXUoxOBFi19VUFKVDOQfozc=",
        version = "v2.2.6",
    )
    go_repository(
        name = "in_gopkg_check_v1",
        importpath = "gopkg.in/check.v1",
        sum = "h1:Hei/4ADfdWqJk1ZMxUNpqntNwaWcugrBjAiHlqqRiVk=",
        version = "v1.0.0-20201130134442-10cb98267c6c",
    )
    go_repository(
        name = "in_gopkg_errgo_v2",
        importpath = "gopkg.in/errgo.v2",
        sum = "h1:0vLT13EuvQ0hNvakwLuFZ/jYrLp5F3kcWHXdRggjCE8=",
        version = "v2.1.0",
    )
    go_repository(
        name = "in_gopkg_evanphx_json_patch_v5",
        importpath = "gopkg.in/evanphx/json-patch.v5",
        sum = "h1:BMT6KIwBD9CaU91PJCZIe46bDmBWa9ynTQgJIOpfQBk=",
        version = "v5.6.0",
    )
    go_repository(
        name = "in_gopkg_h2non_gock_v1",
        importpath = "gopkg.in/h2non/gock.v1",
        sum = "h1:jBbHXgGBK/AoPVfJh5x4r/WxIrElvbLel8TCZkkZJoY=",
        version = "v1.1.2",
    )
    go_repository(
        name = "in_gopkg_inf_v0",
        importpath = "gopkg.in/inf.v0",
        sum = "h1:73M5CoZyi3ZLMOyDlQh031Cx6N9NDJ2Vvfl76EDAgDc=",
        version = "v0.9.1",
    )
    go_repository(
        name = "in_gopkg_ini_v1",
        importpath = "gopkg.in/ini.v1",
        sum = "h1:Dgnx+6+nfE+IfzjUEISNeydPJh9AXNNsWbGP9KzCsOA=",
        version = "v1.67.0",
    )
    go_repository(
        name = "in_gopkg_natefinch_lumberjack_v2",
        importpath = "gopkg.in/natefinch/lumberjack.v2",
        sum = "h1:bBRl1b0OH9s/DuPhuXpNl+VtCaJXFZ5/uEFST95x9zc=",
        version = "v2.2.1",
    )
    go_repository(
        name = "in_gopkg_yaml_v2",
        importpath = "gopkg.in/yaml.v2",
        sum = "h1:D8xgwECY7CYvx+Y2n4sBz93Jn9JRvxdiyyo8CTfuKaY=",
        version = "v2.4.0",
    )
    go_repository(
        name = "in_gopkg_yaml_v3",
        importpath = "gopkg.in/yaml.v3",
        sum = "h1:fxVm/GzAzEWqLHuvctI91KS9hhNmmWOoWu0XTYJS7CA=",
        version = "v3.0.1",
    )
    go_repository(
        name = "io_etcd_go_bbolt",
        importpath = "go.etcd.io/bbolt",
        sum = "h1:j+zJOnnEjF/kyHlDDgGnVL/AIqIJPq8UoB2GSNfkUfQ=",
        version = "v1.3.7",
    )
    go_repository(
        name = "io_etcd_go_etcd_api_v3",
        importpath = "go.etcd.io/etcd/api/v3",
        sum = "h1:4wSsluwyTbGGmyjJktOf3wFQoTBIURXHnq9n/G/JQHs=",
        version = "v3.5.9",
    )
    go_repository(
        name = "io_etcd_go_etcd_client_pkg_v3",
        importpath = "go.etcd.io/etcd/client/pkg/v3",
        sum = "h1:oidDC4+YEuSIQbsR94rY9gur91UPL6DnxDCIYd2IGsE=",
        version = "v3.5.9",
    )
    go_repository(
        name = "io_etcd_go_etcd_client_v2",
        importpath = "go.etcd.io/etcd/client/v2",
        sum = "h1:YZ2OLi0OvR0H75AcgSUajjd5uqKDKocQUqROTG11jIo=",
        version = "v2.305.9",
    )
    go_repository(
        name = "io_etcd_go_etcd_client_v3",
        importpath = "go.etcd.io/etcd/client/v3",
        sum = "h1:r5xghnU7CwbUxD/fbUtRyJGaYNfDun8sp/gTr1hew6E=",
        version = "v3.5.9",
    )
    go_repository(
        name = "io_etcd_go_etcd_pkg_v3",
        importpath = "go.etcd.io/etcd/pkg/v3",
        sum = "h1:6R2jg/aWd/zB9+9JxmijDKStGJAPFsX3e6BeJkMi6eQ=",
        version = "v3.5.9",
    )
    go_repository(
        name = "io_etcd_go_etcd_raft_v3",
        importpath = "go.etcd.io/etcd/raft/v3",
        sum = "h1:ZZ1GIHoUlHsn0QVqiRysAm3/81Xx7+i2d7nSdWxlOiI=",
        version = "v3.5.9",
    )
    go_repository(
        name = "io_etcd_go_etcd_server_v3",
        importpath = "go.etcd.io/etcd/server/v3",
        sum = "h1:vomEmmxeztLtS5OEH7d0hBAg4cjVIu9wXuNzUZx2ZA0=",
        version = "v3.5.9",
    )
    go_repository(
        name = "io_k8s_api",
        build_file_proto_mode = "disable",
        importpath = "k8s.io/api",
        sum = "h1:8ZBrLjwosLl/NYgv1P7EQLqoO8MGQApnbgH8tu3BMzY=",
        version = "v0.28.4",
    )
    go_repository(
        name = "io_k8s_apiextensions_apiserver",
        build_extra_args = ["-exclude=vendor"],
        build_file_proto_mode = "disable",
        importpath = "k8s.io/apiextensions-apiserver",
        sum = "h1:AZpKY/7wQ8n+ZYDtNHbAJBb+N4AXXJvyZx6ww6yAJvU=",
        version = "v0.28.4",
    )
    go_repository(
        name = "io_k8s_apimachinery",
        build_file_proto_mode = "disable",
        importpath = "k8s.io/apimachinery",
        sum = "h1:zOSJe1mc+GxuMnFzD4Z/U1wst50X28ZNsn5bhgIIao8=",
        version = "v0.28.4",
    )
    go_repository(
        name = "io_k8s_apiserver",
        build_file_proto_mode = "disable",
        importpath = "k8s.io/apiserver",
        sum = "h1:BJXlaQbAU/RXYX2lRz+E1oPe3G3TKlozMMCZWu5GMgg=",
        version = "v0.28.4",
    )
    go_repository(
        name = "io_k8s_cli_runtime",
        importpath = "k8s.io/cli-runtime",
        sum = "h1:IW3aqSNFXiGDllJF4KVYM90YX4cXPGxuCxCVqCD8X+Q=",
        version = "v0.28.4",
    )
    go_repository(
        name = "io_k8s_client_go",
        build_file_proto_mode = "disable",
        importpath = "k8s.io/client-go",
        sum = "h1:Np5ocjlZcTrkyRJ3+T3PkXDpe4UpatQxj85+xjaD2wY=",
        version = "v0.28.4",
    )
    go_repository(
        name = "io_k8s_code_generator",
        importpath = "k8s.io/code-generator",
        sum = "h1:tcOSNIZQvuAvXhOwpbuJkKbAABJQeyCcQBCN/3uI18c=",
        version = "v0.28.4",
    )
    go_repository(
        name = "io_k8s_component_base",
        importpath = "k8s.io/component-base",
        sum = "h1:c/iQLWPdUgI90O+T9TeECg8o7N3YJTiuz2sKxILYcYo=",
        version = "v0.28.4",
    )
    go_repository(
        name = "io_k8s_gengo",
        importpath = "k8s.io/gengo",
        sum = "h1:pWEwq4Asjm4vjW7vcsmijwBhOr1/shsbSYiWXmNGlks=",
        version = "v0.0.0-20230829151522-9cce18d56c01",
    )
    go_repository(
        name = "io_k8s_helm",
        importpath = "k8s.io/helm",
        sum = "h1:Bpn6o1wKLYqKM3+Osh8e+1/K2g/GsQJ4F4yNF2+deao=",
        version = "v2.17.0+incompatible",
    )
    go_repository(
        name = "io_k8s_klog",
        importpath = "k8s.io/klog",
        sum = "h1:Pt+yjF5aB1xDSVbau4VsWe+dQNzA0qv1LlXdC2dF6Q8=",
        version = "v1.0.0",
    )
    go_repository(
        name = "io_k8s_klog_v2",
        importpath = "k8s.io/klog/v2",
        sum = "h1:U/Af64HJf7FcwMcXyKm2RPM22WZzyR7OSpYj5tg3cL0=",
        version = "v2.110.1",
    )
    go_repository(
        name = "io_k8s_kms",
        importpath = "k8s.io/kms",
        sum = "h1:PMgY/3CQTWP9eIKmNQiTgjLIZ0ns6O+voagzD2/4mSg=",
        version = "v0.28.4",
    )
    go_repository(
        name = "io_k8s_kube_openapi",
        build_file_proto_mode = "disable",
        importpath = "k8s.io/kube-openapi",
        sum = "h1:ZeIPbyHHqahGIbeyLJJjAUhnxCKqXaDY+n89Ms8szyA=",
        version = "v0.0.0-20231129212854-f0671cc7e66a",
    )
    go_repository(
        name = "io_k8s_sigs_apiserver_network_proxy_konnectivity_client",
        importpath = "sigs.k8s.io/apiserver-network-proxy/konnectivity-client",
        sum = "h1:trsWhjU5jZrx6UvFu4WzQDrN7Pga4a7Qg+zcfcj64PA=",
        version = "v0.1.2",
    )
    go_repository(
        name = "io_k8s_sigs_controller_runtime",
        build_extra_args = ["-exclude=vendor"],
        importpath = "sigs.k8s.io/controller-runtime",
        sum = "h1:2TuvuokmfXvDUamSx1SuAOO3eTyye+47mJCigwG62c4=",
        version = "v0.16.3",
    )
    go_repository(
        name = "io_k8s_sigs_json",
        importpath = "sigs.k8s.io/json",
        sum = "h1:EDPBXCAspyGV4jQlpZSudPeMmr1bNJefnuqLsRAsHZo=",
        version = "v0.0.0-20221116044647-bc3834ca7abd",
    )
    go_repository(
        name = "io_k8s_sigs_kind",
        importpath = "sigs.k8s.io/kind",
        sum = "h1:CScmGz/wX66puA06Gj8OZb76Wmk7JIjgWf5JDvY7msM=",
        version = "v0.17.0",
    )
    go_repository(
        name = "io_k8s_sigs_kustomize_api",
        importpath = "sigs.k8s.io/kustomize/api",
        sum = "h1:6Ca88kEOBVotHDw+y2IsIMYtg9Pvv7MKpW9JMyF/OH4=",
        version = "v0.15.0",
    )
    go_repository(
        name = "io_k8s_sigs_kustomize_kyaml",
        importpath = "sigs.k8s.io/kustomize/kyaml",
        sum = "h1:ynlLMAxDhrY9otSg5GYE2TcIz31XkGZ2Pkj7SdolD84=",
        version = "v0.15.0",
    )
    go_repository(
        name = "io_k8s_sigs_structured_merge_diff_v4",
        importpath = "sigs.k8s.io/structured-merge-diff/v4",
        sum = "h1:150L+0vs/8DA78h1u02ooW1/fFq/Lwr+sGiqlzvrtq4=",
        version = "v4.4.1",
    )
    go_repository(
        name = "io_k8s_sigs_yaml",
        importpath = "sigs.k8s.io/yaml",
        sum = "h1:Mk1wCc2gy/F0THH0TAp1QYyJNzRm2KCLy3o5ASXVI5E=",
        version = "v1.4.0",
    )
    go_repository(
        name = "io_k8s_utils",
        importpath = "k8s.io/utils",
        sum = "h1:FepOBzJ0GXm8t0su67ln2wAZjbQ6RxQGZDnzuLcrUTI=",
        version = "v0.0.0-20231127182322-b307cd553661",
    )
    go_repository(
        name = "io_opencensus_go",
        importpath = "go.opencensus.io",
        sum = "h1:y73uSU6J157QMP2kn2r30vwW1A2W2WFwSCGnAVxeaD0=",
        version = "v0.24.0",
    )
    go_repository(
        name = "io_opencensus_go_contrib_exporter_prometheus",
        importpath = "contrib.go.opencensus.io/exporter/prometheus",
        sum = "h1:sqfsYl5GIY/L570iT+l93ehxaWJs2/OwXtiWwew3oAg=",
        version = "v0.4.2",
    )
    go_repository(
        name = "io_opencensus_go_contrib_exporter_stackdriver",
        importpath = "contrib.go.opencensus.io/exporter/stackdriver",
        sum = "h1:zBakwHardp9Jcb8sQHcHpXy/0+JIb1M8KjigCJzx7+4=",
        version = "v0.13.14",
    )
    go_repository(
        name = "io_opentelemetry_go_collector_pdata",
        importpath = "go.opentelemetry.io/collector/pdata",
        sum = "h1:qCPXSQCoD3qeWFb1RuIks8fw9Atxpk78bmtVdi15KhE=",
        version = "v1.0.0-rcv0016",
    )
    go_repository(
        name = "io_opentelemetry_go_collector_semconv",
        importpath = "go.opentelemetry.io/collector/semconv",
        sum = "h1:BsG1jdLLRCBRlvUujk4QA86af7r/ZXnizczQpEs/gg8=",
        version = "v0.87.0",
    )
    go_repository(
        name = "io_opentelemetry_go_contrib_instrumentation_google_golang_org_grpc_otelgrpc",
        importpath = "go.opentelemetry.io/contrib/instrumentation/google.golang.org/grpc/otelgrpc",
        sum = "h1:xFSRQBbXF6VvYRf2lqMJXxoB72XI1K/azav8TekHHSw=",
        version = "v0.35.0",
    )
    go_repository(
        name = "io_opentelemetry_go_contrib_instrumentation_net_http_otelhttp",
        importpath = "go.opentelemetry.io/contrib/instrumentation/net/http/otelhttp",
        sum = "h1:x8Z78aZx8cOF0+Kkazoc7lwUNMGy0LrzEMxTm4BbTxg=",
        version = "v0.45.0",
    )
    go_repository(
        name = "io_opentelemetry_go_otel",
        importpath = "go.opentelemetry.io/otel",
        sum = "h1:MuS/TNf4/j4IXsZuJegVzI1cwut7Qc00344rgH7p8bs=",
        version = "v1.19.0",
    )
    go_repository(
        name = "io_opentelemetry_go_otel_exporters_otlp_internal_retry",
        importpath = "go.opentelemetry.io/otel/exporters/otlp/internal/retry",
        sum = "h1:TaB+1rQhddO1sF71MpZOZAuSPW1klK2M8XxfrBMfK7Y=",
        version = "v1.10.0",
    )
    go_repository(
        name = "io_opentelemetry_go_otel_exporters_otlp_otlptrace",
        importpath = "go.opentelemetry.io/otel/exporters/otlp/otlptrace",
        sum = "h1:Mne5On7VWdx7omSrSSZvM4Kw7cS7NQkOOmLcgscI51U=",
        version = "v1.19.0",
    )
    go_repository(
        name = "io_opentelemetry_go_otel_exporters_otlp_otlptrace_otlptracegrpc",
        importpath = "go.opentelemetry.io/otel/exporters/otlp/otlptrace/otlptracegrpc",
        sum = "h1:3d+S281UTjM+AbF31XSOYn1qXn3BgIdWl8HNEpx08Jk=",
        version = "v1.19.0",
    )
    go_repository(
        name = "io_opentelemetry_go_otel_exporters_otlp_otlptrace_otlptracehttp",
        importpath = "go.opentelemetry.io/otel/exporters/otlp/otlptrace/otlptracehttp",
        sum = "h1:IeMeyr1aBvBiPVYihXIaeIZba6b8E1bYp7lbdxK8CQg=",
        version = "v1.19.0",
    )
    go_repository(
        name = "io_opentelemetry_go_otel_metric",
        importpath = "go.opentelemetry.io/otel/metric",
        sum = "h1:aTzpGtV0ar9wlV4Sna9sdJyII5jTVJEvKETPiOKwvpE=",
        version = "v1.19.0",
    )
    go_repository(
        name = "io_opentelemetry_go_otel_sdk",
        importpath = "go.opentelemetry.io/otel/sdk",
        sum = "h1:6USY6zH+L8uMH8L3t1enZPR3WFEmSTADlqldyHtJi3o=",
        version = "v1.19.0",
    )
    go_repository(
        name = "io_opentelemetry_go_otel_trace",
        importpath = "go.opentelemetry.io/otel/trace",
        sum = "h1:DFVQmlVbfVeOuBRrwdtaehRrWiL1JoVs9CPIQ1Dzxpg=",
        version = "v1.19.0",
    )
    go_repository(
        name = "io_opentelemetry_go_proto_otlp",
        importpath = "go.opentelemetry.io/proto/otlp",
        sum = "h1:T0TX0tmXU8a3CbNXzEKGeU5mIVOdf0oykP+u2lIVU/I=",
        version = "v1.0.0",
    )
    go_repository(
        name = "io_rsc_binaryregexp",
        importpath = "rsc.io/binaryregexp",
        sum = "h1:HfqmD5MEmC0zvwBuF187nq9mdnXjXsSivRiXN7SmRkE=",
        version = "v0.2.0",
    )
    go_repository(
        name = "io_rsc_quote_v3",
        importpath = "rsc.io/quote/v3",
        sum = "h1:9JKUTTIUgS6kzR9mK1YuGKv6Nl+DijDNIc0ghT58FaY=",
        version = "v3.1.0",
    )
    go_repository(
        name = "io_rsc_sampler",
        importpath = "rsc.io/sampler",
        sum = "h1:7uVkIFmeBqHfdjD+gZwtXXI+RODJ2Wc4O7MPEh/QiW4=",
        version = "v1.3.0",
    )
    go_repository(
        name = "net_starlark_go",
        importpath = "go.starlark.net",
        sum = "h1:VdD38733bfYv5tUZwEIskMM93VanwNIi5bIKnDrJdEY=",
        version = "v0.0.0-20230525235612-a134d8f9ddca",
    )
    go_repository(
        name = "org_golang_google_api",
        importpath = "google.golang.org/api",
        sum = "h1:b2CqT6kG+zqJIVKRQ3ELJVLN1PwHZ6DJ3dW8yl82rgY=",
        version = "v0.149.0",
    )
    go_repository(
        name = "org_golang_google_appengine",
        importpath = "google.golang.org/appengine",
        sum = "h1:IhEN5q69dyKagZPYMSdIjS2HqprW324FRQZJcGqPAsM=",
        version = "v1.6.8",
    )
    go_repository(
        name = "org_golang_google_genproto",
        importpath = "google.golang.org/genproto",
        sum = "h1:W12Pwm4urIbRdGhMEg2NM9O3TWKjNcxQhs46V0ypf/k=",
        version = "v0.0.0-20231127180814-3a041ad873d4",
    )
    go_repository(
        name = "org_golang_google_genproto_googleapis_api",
        importpath = "google.golang.org/genproto/googleapis/api",
        sum = "h1:ZcOkrmX74HbKFYnpPY8Qsw93fC29TbJXspYKaBkSXDQ=",
        version = "v0.0.0-20231127180814-3a041ad873d4",
    )
    go_repository(
        name = "org_golang_google_genproto_googleapis_bytestream",
        importpath = "google.golang.org/genproto/googleapis/bytestream",
        sum = "h1:o4S3HvTUEXgRsNSUQsALDVog0O9F/U1JJlHmmUN8Uas=",
        version = "v0.0.0-20231030173426-d783a09b4405",
    )
    go_repository(
        name = "org_golang_google_genproto_googleapis_rpc",
        importpath = "google.golang.org/genproto/googleapis/rpc",
        sum = "h1:DC7wcm+i+P1rN3Ff07vL+OndGg5OhNddHyTA+ocPqYE=",
        version = "v0.0.0-20231127180814-3a041ad873d4",
    )
    go_repository(
        name = "org_golang_google_grpc",
        importpath = "google.golang.org/grpc",
        sum = "h1:Z5Iec2pjwb+LEOqzpB2MR12/eKFhDPhuqW91O+4bwUk=",
        version = "v1.59.0",
    )
    go_repository(
        name = "org_golang_google_protobuf",
        importpath = "google.golang.org/protobuf",
        sum = "h1:g0LDEJHgrBl9N9r17Ru3sqWhkIx2NB67okBHPwC7hs8=",
        version = "v1.31.0",
    )
    go_repository(
        name = "org_golang_x_crypto",
        importpath = "golang.org/x/crypto",
        sum = "h1:mMMrFzRSCF0GvB7Ne27XVtVAaXLrPmgPC7/v0tkwHaY=",
        version = "v0.16.0",
    )
    go_repository(
        name = "org_golang_x_exp",
        importpath = "golang.org/x/exp",
        sum = "h1:Gvh4YaCaXNs6dKTlfgismwWZKyjVZXwOPfIyUaqU3No=",
        version = "v0.0.0-20231127185646-65229373498e",
    )
    go_repository(
        name = "org_golang_x_image",
        importpath = "golang.org/x/image",
        sum = "h1:+qEpEAPhDZ1o0x3tHzZTQDArnOixOzGD9HUJfcg0mb4=",
        version = "v0.0.0-20190802002840-cff245a6509b",
    )
    go_repository(
        name = "org_golang_x_lint",
        importpath = "golang.org/x/lint",
        sum = "h1:VLliZ0d+/avPrXXH+OakdXhpJuEoBZuwh1m2j7U6Iug=",
        version = "v0.0.0-20210508222113-6edffad5e616",
    )
    go_repository(
        name = "org_golang_x_mobile",
        importpath = "golang.org/x/mobile",
        sum = "h1:4+4C/Iv2U4fMZBiMCc98MG1In4gJY5YRhtpDNeDeHWs=",
        version = "v0.0.0-20190719004257-d2bd2a29d028",
    )
    go_repository(
        name = "org_golang_x_mod",
        importpath = "golang.org/x/mod",
        sum = "h1:dGoOF9QVLYng8IHTm7BAyWqCqSheQ5pYWGhzW00YJr0=",
        version = "v0.14.0",
    )
    go_repository(
        name = "org_golang_x_net",
        importpath = "golang.org/x/net",
        sum = "h1:zTwKpTd2XuCqf8huc7Fo2iSy+4RHPd10s4KzeTnVr1c=",
        version = "v0.19.0",
    )
    go_repository(
        name = "org_golang_x_oauth2",
        importpath = "golang.org/x/oauth2",
        sum = "h1:s8pnnxNVzjWyrvYdFUQq5llS1PX2zhPXmccZv99h7uQ=",
        version = "v0.15.0",
    )
    go_repository(
        name = "org_golang_x_sync",
        importpath = "golang.org/x/sync",
        sum = "h1:60k92dhOjHxJkrqnwsfl8KuaHbn/5dl0lUPUklKo3qE=",
        version = "v0.5.0",
    )
    go_repository(
        name = "org_golang_x_sys",
        importpath = "golang.org/x/sys",
        sum = "h1:h48lPFYpsTvQJZF4EKyI4aLHaev3CxivZmv7yZig9pc=",
        version = "v0.15.0",
    )
    go_repository(
        name = "org_golang_x_term",
        importpath = "golang.org/x/term",
        sum = "h1:y/Oo/a/q3IXu26lQgl04j/gjuBDOBlx7X6Om1j2CPW4=",
        version = "v0.15.0",
    )
    go_repository(
        name = "org_golang_x_text",
        importpath = "golang.org/x/text",
        sum = "h1:ScX5w1eTa3QqT8oi6+ziP7dTV1S2+ALU0bI+0zXKWiQ=",
        version = "v0.14.0",
    )
    go_repository(
        name = "org_golang_x_time",
        importpath = "golang.org/x/time",
        sum = "h1:o7cqy6amK/52YcAKIPlM3a+Fpj35zvRj2TP+e1xFSfk=",
        version = "v0.5.0",
    )

    # TODO(ensonic): don't update until we have a new gazelle release
    # https://github.com/bazelbuild/rules_go/issues/2910
    go_repository(
        name = "org_golang_x_tools",
        build_directives = [
            "gazelle:exclude **/testdata",
        ],
        importpath = "golang.org/x/tools",
        sum = "h1:GO788SKMRunPIBCXiQyo2AaexLstOrVhuAL5YwsckQM=",
        version = "v0.16.0",
    )
    go_repository(
        name = "org_golang_x_xerrors",
        importpath = "golang.org/x/xerrors",
        sum = "h1:H2TDz8ibqkAF6YGhCdN3jS9O0/s90v0rJh3X/OLHEUk=",
        version = "v0.0.0-20220907171357-04be3eba64a2",
    )
    go_repository(
        name = "org_mongodb_go_mongo_driver",
        importpath = "go.mongodb.org/mongo-driver",
        sum = "h1:aPx33jmn/rQuJXPQLZQ8NtfPQG8CaqgLThFtqRb0PiE=",
        version = "v1.12.0",
    )
    go_repository(
        name = "org_uber_go_atomic",
        importpath = "go.uber.org/atomic",
        sum = "h1:ZvwS0R+56ePWxUNi+Atn9dWONBPp/AUETXlHW0DxSjE=",
        version = "v1.11.0",
    )
    go_repository(
        name = "org_uber_go_automaxprocs",
        importpath = "go.uber.org/automaxprocs",
        sum = "h1:kWazyxZUrS3Gs4qUpbwo5kEIMGe/DAvi5Z4tl2NW4j8=",
        version = "v1.5.3",
    )
    go_repository(
        name = "org_uber_go_goleak",
        importpath = "go.uber.org/goleak",
        sum = "h1:2K3zAYmnTNqV73imy9J1T3WC+gmCePx2hEGkimedGto=",
        version = "v1.3.0",
    )
    go_repository(
        name = "org_uber_go_multierr",
        importpath = "go.uber.org/multierr",
        sum = "h1:blXXJkSxSSfBVBlC76pxqeO+LN3aDfLQo+309xJstO0=",
        version = "v1.11.0",
    )
    go_repository(
        name = "org_uber_go_zap",
        importpath = "go.uber.org/zap",
        sum = "h1:sI7k6L95XOKS281NhVKOFCUNIvv9e0w4BF8N3u+tCRo=",
        version = "v1.26.0",
    )
    go_repository(
        name = "tools_gotest_v3",
        importpath = "gotest.tools/v3",
        sum = "h1:4AuOwCGf4lLR9u3YOe2awrHygurzhO/HeQ6laiA6Sx0=",
        version = "v3.0.3",
    )
    go_repository(
        name = "xyz_gomodules_jsonpatch_v2",
        importpath = "gomodules.xyz/jsonpatch/v2",
        sum = "h1:Ci3iUJyx9UeRx7CeFN8ARgGbkESwJK+KB9lLcWxY/Zw=",
        version = "v2.4.0",
    )
