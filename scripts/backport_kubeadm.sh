#!/usr/bin/env bash
#
# Copyright 2019 The Google Cloud Robotics Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Backports the kubelet and kubeadm packages from Ubuntu 16 (xenial) to Ubuntu
# 14 (trusty).
#
# Essentially, the backport only does one thing: it works around the issue that Ubuntu 16 uses
# systemd as init system and Ubuntu 14 uses upstart. This is problematic because kubelet runs as a
# service.
# We replace the systemd service definition for kubelet with an upstart service definition and use
# upstart commands to start/stop kubelet. Additionally, since kubeadm issues systemctl commands at
# runtime, we add a simple "systemctl" script to /usr/bin in the kubeadm package which translates
# systemctl commands into upstart commands.

set -eux

WORKSPACE_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && cd .. && pwd )"

K8S_VERSION="1.12.2"
K8S_PKG_VERSION="${K8S_VERSION}-00"
BACKPORT_VERSION_POSTFIX="~ubuntu14.04.1"

K8S_REPOSITORY="https://packages.cloud.google.com/apt"
K8S_PACKAGES_URL="${K8S_REPOSITORY}/dists/kubernetes-xenial/main/binary-amd64/Packages"

function replace_in_file {
  local file="$1"
  local pattern="$2"
  local replacement="$3"

  sed "s/${pattern}/${replacement}/g" -i "${file}"
}

function build_nsenter_if_not_exists {
  local nsenter_target_path="$1"

  if [[ ! -f "${nsenter_target_path}" ]]; then
    echo "Building nsenter in an Ubuntu 14 docker container..."

    # Start new docker container with Ubuntu 14
    docker create --name nsenter -it ubuntu:14.04
    docker start nsenter

    # Download and build util-linux which contains the nsenter binary
    docker exec nsenter apt-get update
    docker exec nsenter apt-get install -y git build-essential bison libncurses5-dev libslang2-dev \
      gettext zlib1g-dev libselinux1-dev debhelper lsb-release pkg-config po-debconf autoconf \
      automake autopoint libtool

    docker exec nsenter git clone git://git.kernel.org/pub/scm/utils/util-linux/util-linux.git \
      util-linux

    docker exec -w /util-linux nsenter ./autogen.sh
    docker exec -w /util-linux nsenter ./configure \
      --without-python --disable-all-programs --enable-nsenter
    docker exec -w /util-linux nsenter make

    # Copy out of container
    docker cp nsenter:/util-linux/nsenter "${nsenter_target_path}"

    # Cleanup container
    docker stop nsenter
    docker rm nsenter
  fi
}

function download_orig_pkg {
  local package_name="$1"

  # Adding external repositories is not allowed on all machines, which prevents us from doing
  # something simple such as "aptitude download kubelet". Thus we manually curl the .deb file from
  # the repository after getting its location from the appropriate Packages file.
  local path=$(curl -fsSL "$K8S_PACKAGES_URL" \
    | grep "${package_name}_${K8S_PKG_VERSION}" \
    | cut -d ' ' -f2)
  local filename=$(basename "${path}")

  curl -fsSL "${K8S_REPOSITORY}/${path}" --output "${filename}"

  echo "${filename}"
}

function unpack_deb {
  local deb_file="$1"
  local unpack_dirname="$2"

  mkdir -p "${unpack_dirname}"
  pushd "${unpack_dirname}"

  ar x "../${deb_file}"
  mkdir control.orig data.orig
  tar --directory=control.orig -xf control.tar.gz
  tar --directory=data.orig -xf data.tar.xz
  rm control.tar.gz data.tar.xz
  cp -r data.orig data
  cp -r control.orig control

  popd
}

function repack_deb {
  local unpacked_dirname="$1"
  local deb_file="$2"

  pushd "${unpacked_dirname}"

  # Manually invoke xz since the --xz option for tar is horribly slow in comparison.
  tar --directory=data -cf data.tar .
  xz -v0 data.tar # replaces "data.tar" with "data.tar.xz"

  tar --directory=control --gzip -cf control.tar.gz .
  ar cr "../${deb_file}" debian-binary control.tar.gz data.tar.xz
  rm data.tar.xz
  rm control.tar.gz

  popd
}

function do_nothing_script {
  cat <<'EOF'
#!/bin/sh
exit 0
EOF
}

function stop_kubelet_script {
  cat <<'EOF'
#!/bin/sh
stop kubelet || true
exit 0
EOF
}

function restart_kubelet_script {
  cat <<'EOF'
#!/bin/sh
if [ "\$1" = configure ]; then
  start kubelet || restart kubelet
fi
exit 0
EOF
}

function systemctl_script_delegating_to_upstart {
  cat <<'EOF'
#!/usr/bin/env bash

# This script was added by the package "kubeadm" (more precisely, a backported version of this
# package for Ubuntu versions <=14). This is not a full replacement for systemctl. This just adds
# the minimum required support for the systemctl commands issued by kubeadm at runtime.

set -eu

function daemon-reload {
  exit 0
}

function start {
  local service=$1

  command start ${service} || true
}

function restart {
  local service=$1

  command start ${service} || command restart ${service}
}

function stop {
  local service=$1

  command stop ${service} || true
}

function status {
  local service=$1

  # kubeadm specifically looks for the string "Loaded: not-found" (see
  # https://github.com/kubernetes/kubernetes/blob/7f23a743e8c23ac6489340bbb34fa6f1d392db9d/pkg/util/initsystem/initsystem.go#L81)
  (command status "${service}" &>/dev/null) && echo "Service exists" || echo "Loaded: not-found"
}

function is-enabled {
  exit 0
}

function is-active {
  local service=$1

  # kubeadm specifically looks for the string "active" (see
  # https://github.com/kubernetes/kubernetes/blob/7f23a743e8c23ac6489340bbb34fa6f1d392db9d/pkg/util/initsystem/initsystem.go#L104
  (command status "${service}" | grep running &>/dev/null) && echo "active" || echo "off"
}

if [ "$#" -lt 1 ]; then
  message="This is a substitute systemctl which delegates to upstart commands and only supports a\
 small subset of commands. Usage: $0 {daemon-reload|start|stop|status|is-enabled} <service>"
  echo "${message}" >&2
  exit 1
fi

# call arguments verbatim:
$@
EOF
}

function kubelet_upstart_conf {
# TODO(ferstl): KUBELET_KUBEADM_ARGS and KUBELET_EXTRA_ARGS should be parsed
# on-the-fly from /var/lib/kubelet/kubeadm-flags.env and /etc/default/kubelet rather than being
# hardcoded. This would then correspond to the actual behavior of the following two lines in
# the systemd service config where this is converted from:
#   EnvironmentFile=-/var/lib/kubelet/kubeadm-flags.env
#   EnvironmentFile=-/etc/default/kubelet
  cat <<'EOF'
description "Kubelet"
start on (docker)
stop on runlevel [!2345]
limit nproc unlimited unlimited
respawn
respawn limit unlimited
kill timeout 30
script
        KUBELET_KUBECONFIG_ARGS="--bootstrap-kubeconfig=/etc/kubernetes/bootstrap-kubelet.conf --kubeconfig=/etc/kubernetes/kubelet.conf"
        KUBELET_CONFIG_ARGS="--config=/var/lib/kubelet/config.yaml"
        KUBELET_KUBEADM_ARGS="--network-plugin= --node-ip=192.168.9.1 --resolv-conf=/etc/kubernetes/resolv.conf --cgroup-driver=cgroupfs"
        KUBELET_EXTRA_ARGS=
        exec /usr/bin/kubelet $KUBELET_KUBECONFIG_ARGS $KUBELET_CONFIG_ARGS $KUBELET_KUBEADM_ARGS $KUBELET_EXTRA_ARGS
end script
EOF
}

function patch_kubelet {
  # Note: The original kubelet package defines a systemd service which just runs kubelet without
  # any arguments, and the original kubeadm package adds a systemd drop-in for kubelet which changes
  # the kubelet service to call the kubelet binary with various command-line params.
  # We could do something similar for upstart (basic kubelet.conf in kubelet package, additional
  # kubelet.conf.override in kubeadm package), but don't worry about that for now. We just put the
  # final kubelet.conf into the kubeadm package below. The backported kubelet packages thus cannot
  # start the kubelet service upon install/remove since it is not defined at that point.

  # --- Package control files (metadata and install scripts)
  rm control/md5sums
  do_nothing_script > control/postinst
  stop_kubelet_script > control/prerm
  do_nothing_script > control/postrm
  replace_in_file control/control \
    "Version: ${K8S_PKG_VERSION}" \
    "Version: ${K8S_PKG_VERSION}${BACKPORT_VERSION_POSTFIX}"
  replace_in_file control/control \
    "Maintainer: Kubernetes Authors <kubernetes-dev+release@googlegroups.com>" \
    "Maintainer: Cloud Robotics Team <cloud-robotics-discuss@googlegroups.com>"
  # Change the version for the dependency on init-system-helpers, i.e., do not require systemd.
  # This replaces, e.g., ">= 1.18~" with ">= 1.14~".
  replace_in_file control/control \
    "init-system-helpers ([^()]*)" \
    "init-system-helpers (>= 1.14~)"

  # --- Package data (files installed on system)
  rm -r data/lib # deletes the kubelet systemd service config
}

function patch_kubeadm {
  # --- Package control files (metadata and install scripts)
  rm control/md5sums
  restart_kubelet_script > control/postinst
  echo "/etc/init/kubelet.conf" > control/conffiles
  replace_in_file control/control \
    "Version: ${K8S_PKG_VERSION}" \
    "Version: ${K8S_PKG_VERSION}${BACKPORT_VERSION_POSTFIX}"
  replace_in_file control/control \
    "Maintainer: Kubernetes Authors <kubernetes-dev+release@googlegroups.com>" \
    "Maintainer: Cloud Robotics Team <cloud-robotics-discuss@googlegroups.com>"
  # Make the backported kubeadm package depend on the backported kubelet package.
  # This replaces, e.g., ">= 1.12.2-00" with "= 1.12.2-00~ubuntu14.04.1".
  replace_in_file control/control \
    "kubelet ([^()]*)" \
    "kubelet (= ${K8S_PKG_VERSION}${BACKPORT_VERSION_POSTFIX})"

  # --- Package data (files installed on system)
  # The nsenter binary is expected by kubeadm but not present in Ubuntu versions <= 14. There is no
  # package on which we could depend for this, so we just add the binary directly.
  build_nsenter_if_not_exists ${WORKSPACE_ROOT}/nsenter
  cp ${WORKSPACE_ROOT}/nsenter data/usr/bin/nsenter

  # kubeadm issues systemctl commands at runtime. Add a script to forward these to upstart commands.
  # E.g., "systemctl start kubelet" is mapped to "start kubelet".
  systemctl_script_delegating_to_upstart > data/usr/bin/systemctl
  chmod +x data/usr/bin/systemctl

  # This configures the kubelet service from scratch (see note in patch_kubelet()).
  rm -r data/etc/systemd/
  mkdir -p data/etc/init/
  kubelet_upstart_conf > data/etc/init/kubelet.conf
}

function backport_kubelet_and_kubeadm {
  mkdir -p ${WORKSPACE_ROOT}/backport
  rm -f -r ${WORKSPACE_ROOT}/backport/*
  pushd ${WORKSPACE_ROOT}/backport

  local kubelet_deb=$(download_orig_pkg kubelet)
  local kubelet_dirname=${kubelet_deb/.deb/}
  unpack_deb ${kubelet_deb} ${kubelet_dirname}
  (cd ${kubelet_dirname} && patch_kubelet)
  repack_deb ${kubelet_dirname} "backported_kubelet.deb"

  local kubeadm_deb=$(download_orig_pkg kubeadm)
  local kubeadm_dirname=${kubeadm_deb/.deb/}
  unpack_deb ${kubeadm_deb} ${kubeadm_dirname}
  (cd ${kubeadm_dirname} && patch_kubeadm)
  repack_deb ${kubeadm_dirname} "backported_kubeadm.deb"

  popd
}

if [[ "$#" -lt 1 ]]; then
  backport_kubelet_and_kubeadm
else
  "$@" # call arguments verbatim
fi
