#!/bin/bash
#
# Copyright 2019 The Cloud Robotics Authors
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

# This script contains the functions to install all dependencies required to run k8s on the robot
# (mainly docker and kubeadm) and to get the bare k8s cluster running. It can also be executed
# directly if one is only interested in getting the bare k8s cluster to run without doing the full
# robot setup.

# kubeadm 1.12.2 doesn't support docker-ce 18.09.0 or higher yet (as of 11/2018).
DOCKER_VERSION="18.06.1"
DOCKER_PACKAGE_VERSION="${DOCKER_VERSION}~ce~3-0~ubuntu"
K8S_VERSION="1.12.2"

# The IP address of the host system on Docker's docker0 bridge network, along with the netmask for
# the subnet. These are depended on in multiple places, including the allowed subnet in
# metadata-server.yaml.
BRIDGE_IP="192.168.9.1/24"

function check_distribution_is_supported {
  if [[ "$(lsb_release -is)" != "Ubuntu" ]] ; then
    echo "ERROR: This script requires Ubuntu, but it detected $(lsb_release -is)." >&2
    return 1
  fi
  if [[ ! "$(lsb_release -rs)" =~ ^1[468].04$ ]] ; then
    echo "ERROR: This script only supports Ubuntu 14.04, 16.04, and 18.04, but it" >&2
    echo "detected $(lsb_release -rs)." >&2
    return 1
  fi
}

function add_apt_key {
  local key_url=$1

  curl -fsSL "${key_url}" | sudo apt-key add -
}

# apt_install tries to install a package non-interactively. If it fails, it prints a message to
# prompt the user to install it interactively, which may be required if eg config files have
# conflicts, or if the package must be downgraded. An alternative would be to provide
# --allow-downgrades --allow-remove-essential --allow-change-held-packages, but these are
# dangerous, so we give the user a chance to avoid damage to the system.
function apt_install {
  echo "Installing $*..."
  if ! sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"; then
    echo ""
    echo "ERROR: Failed to install $*. Try:" >&2
    echo "    sudo apt install $*" >&2
    return 1
  fi
}

function install_common_deps {
  sudo apt-get update
  apt_install \
        apt-transport-https \
        ca-certificates \
        curl \
        software-properties-common # "apt-add-repository" command
}

function install_docker_deps {
  # Install docker if necessary
  if ! docker --version 2>/dev/null | grep -qF "${DOCKER_VERSION}" ; then
    echo "Preparing to install Docker..."
    add_apt_key https://download.docker.com/linux/ubuntu/gpg
    sudo add-apt-repository \
       "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
       $(lsb_release -cs) \
       stable"
    sudo apt-get update
    apt_install docker-ce="${DOCKER_PACKAGE_VERSION}"
  fi
}

function install_k8s_deps {
  # Add k8s repo if necessary
  if [[ ! -f /etc/apt/sources.list.d/kubernetes.list ]] ; then
    echo "Preparing to install Kubernetes..."
    add_apt_key https://packages.cloud.google.com/apt/doc/apt-key.gpg
    sudo tee /etc/apt/sources.list.d/kubernetes.list >/dev/null \
      <<< "deb http://apt.kubernetes.io/ kubernetes-xenial main"

    if [[ $(lsb_release -cs) = "trusty" ]]; then
      add_apt_key https://cloud-robotics-packages.storage.googleapis.com/doc/gpg-public.key
      sudo tee -a /etc/apt/sources.list.d/kubernetes.list >/dev/null \
        <<<  "deb https://cloud-robotics-packages.storage.googleapis.com/ trusty main"
    fi

    sudo apt-get update
  fi

  # If ubuntu version <= trusty (=14.04), use the backported packages from the packages
  # repository (only required for kubelet and kubeadm, kubectl is compatible with trusty).
  local version_postfix=""
  if [[ $(lsb_release -cs) = "trusty" ]]; then
    version_postfix="~ubuntu14.04.1"
  fi

  # Install or upgrade k8s binaries
  if ! kubectl version --client 2>/dev/null | grep -qF "${K8S_VERSION}" ; then
    apt_install "kubectl=${K8S_VERSION}-00"
  fi
  if ! kubelet --version 2>/dev/null | grep -qF "${K8S_VERSION}" ; then
    apt_install "kubelet=${K8S_VERSION}-00${version_postfix}"
  fi
  if ! kubeadm version 2>/dev/null | grep -qF "${K8S_VERSION}" ; then
    apt_install "kubeadm=${K8S_VERSION}-00${version_postfix}"

    # Remove the local cluster, so that it can be safely (re)installed by
    # setup_cluster below.
    # TODO(rodrigoq): this is destructive, as it deletes etcd's datadir. Check
    # if etcd preserves/migrates the datadir across releases.
    # TODO(rodrigoq): detect if kubeadm.yaml changes and also do this.
    # TODO(ferstl): Once we move the k8s-on-robot bootstrapping to a debian package, this has to be
    # done by an install or post-install script and we need to figure out another way to determine
    # whether the kubeadm was updated or not.
    echo "Deleting the old local cluster..."
    sudo kubeadm reset --force
  fi
}

function partial_docker_daemon_config_for_bridge_ip {
  echo "\"bip\": \"${BRIDGE_IP}\""
}

function docker_daemon_config {
  cat << EOF
{
  $(partial_docker_daemon_config_for_bridge_ip),
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  }
}
EOF
}

function restart_system_service {
  local service="$1"

  if [[ $(lsb_release -cs) = "trusty" ]]; then
    sudo restart "${service}"
  else
    sudo systemctl restart "${service}"
  fi
}

function setup_docker {
  if sudo test ! -f /etc/docker/daemon.json; then
    echo "Configuring the docker daemon..."
    docker_daemon_config | sudo tee /etc/docker/daemon.json
    restart_system_service docker
  else
    # Docker daemon config already exists. Check it sets the bridge IP.
    if ! sudo grep -q "$(partial_docker_daemon_config_for_bridge_ip)" /etc/docker/daemon.json; then
      echo "ERROR: /etc/docker/daemon.json doesn't set $(partial_docker_daemon_config_for_bridge_ip)" >&2
      echo "Please manually apply this setting to /etc/docker/daemon.json." >&2
      return 1
    fi
    # Check if the log size is limited. Otherwise, the system will have problems with disk space
    # and/or fluentd performance.
    if ! sudo grep -q "\"max-size\"" /etc/docker/daemon.json; then
      echo "ERROR: /etc/docker/daemon.json doesn't enable log rotation." >&2
      echo "Please manually apply this to /etc/docker/daemon.json. See:" >&2
      echo "    https://success.docker.com/article/how-to-setup-log-rotation-post-installation" >&2
      return 1
    fi
  fi

  sudo adduser $USER docker
}

function setup_k8s {
  # Install a basic resolv.conf that kubelet can use. The default is
  # /etc/resolv.conf, which often points to a local cache (eg dnsmasq or
  # systemd-resolved) on the loopback address.
  sudo mkdir -p /etc/kubernetes
  sudo tee /etc/kubernetes/resolv.conf >/dev/null << EOF
# resolv.conf for kubelet. Automatically created by cloud robotics' install_k8s_on_robot.sh.
nameserver 8.8.8.8
EOF
}

# Kubernetes doesn't support hosts with swap enabled, and it can't intelligently
# share this between containers. Although some people report success running it
# with swap (allowing them to run workloads with spiky RAM usage profile on
# smaller instances), for now we have no need, so we disable swap to make
# kubeadm happy.
function disable_swap {
  echo "Disabling swap on the local system..."

  # Remove swap entries from fstab.
  sudo sed -i '/ swap / s/^/# /' /etc/fstab

  # systemd automatically discovers swap partitions from the partition table and
  # activates them. Repeatedly. Mask the units to work around this charming
  # behavior:
  systemctl list-units --type swap \
    | awk '/ loaded active / {print $1;}' \
    | sudo xargs --no-run-if-empty systemctl mask

  # Temporarily disable swap until the next boot.
  sudo swapoff -a
}

function setup_cluster {
  local kubeadm_yaml=$1

  # This world-readable file is created by "kubeadm init" (and deleted by "kubeadm reset"),
  # so we use it to detect an existing local cluster.
  if [[ -f /etc/kubernetes/pki/ca.crt ]] ; then
    return
  fi

  disable_swap

  echo "Initializing the local cluster..."
  sudo kubeadm init --config "${kubeadm_yaml}"

  # Merge generated kubeconfig into ~/.kube/config
  echo "Adding the local cluster to ~/.kube/config..."
  tmp=$(mktemp)
  # shellcheck disable=2024
  # sudo is required for kubectl to read admin.conf, not to write to $tmp.
  sudo KUBECONFIG=/etc/kubernetes/admin.conf:$HOME/.kube/config \
    kubectl config view --flatten > $tmp
  mkdir --parents ~/.kube
  mv "$tmp" ~/.kube/config
  chmod 600 ~/.kube/config

  # TODO(b/118584427): remove this permissive binding and use proper auth
  echo "Creating a permissive policy for the local cluster..."
  kubectl create clusterrolebinding permissive-binding \
    --clusterrole=cluster-admin \
    --user=admin \
    --user=kubelet \
    --group=system:serviceaccounts
}

function create_default_kubeadm_config {
  local temp_config
  temp_config=$(mktemp -t kubeadm.XXXXXXXX.yaml)
  cat > "${temp_config}" << "EOF"
apiVersion: kubeadm.k8s.io/v1alpha3
kind: InitConfiguration
apiEndpoint:
  # This should match `bind-address` from `apiServerExtraArgs` so that the
  # apiserver is reachable from the advertised IP.
  advertiseAddress: 192.168.9.1
  bindPort: 6443

nodeRegistration:
  kubeletExtraArgs:
    # network-plugin: "" uses the "noop" network plugin, where the docker
    # bridge provides pod networking. This is sufficient for a single-node
    # cluster. Although "noop" is not officially supported by kubeadm (which
    # expects the user to install a CNI plugin) it is explicitly used by
    # minikube and seems to work well.
    network-plugin: ""

    # This should match `address` in the KubeletConfiguration so that the
    # kubelet is reachable under the node IP. Otherwise, `kubectl port-forward`
    # (among other things) won't work.
    node-ip: 192.168.9.1

    # Override resolv.conf to provide DNS resolution independent of
    # /etc/resolv.conf, which may be set to a loopback resolver by
    # NetworkManager or systemd.
    resolv-conf: /etc/kubernetes/resolv.conf

  # Overrides the default taints for the master node, which would have
  # prevented workloads running on the same node as the apiserver etc.
  taints: []
---
apiVersion: kubeadm.k8s.io/v1alpha3
kind: ClusterConfiguration
kubernetesVersion: v1.12.2
apiServerExtraArgs:
  # Bind to the docker interface to avoid problems when external interfaces
  # (ethernet, Wi-Fi) drop out. This also prevents connections to the apiserver
  # from outside the robot.
  bind-address: 192.168.9.1
controllerManagerExtraArgs:
  # Bind to the docker interface to prevent connections from outside the robot.
  bind-address: 192.168.9.1
---
apiVersion: kubeproxy.config.k8s.io/v1alpha1
kind: KubeProxyConfiguration
# Bind to the docker interface to prevent connections from outside the robot.
bindAddress: 192.168.9.1
healthzBindAddress: 192.168.9.1
---
apiVersion: kubelet.config.k8s.io/v1beta1
kind: KubeletConfiguration
# Bind to the docker interface to prevent connections from outside the robot.
address: 192.168.9.1
EOF
  echo "${temp_config}"
}

function install_k8s_on_robot {
  local kubeadm_yaml=$1

  check_distribution_is_supported

  # Install required dependencies. Once we move the k8s-on-robot bootstrapping to a debian package,
  # all of the following package installs should be enforced via package dependencies.
  install_common_deps
  install_docker_deps
  install_k8s_deps

  # Setup and configure docker and k8s.
  setup_docker
  setup_k8s
  setup_cluster "${kubeadm_yaml}"

  echo
  echo "The local Kubernetes cluster has been installed."
}

function main {
  if [[ "$EUID" -eq 0 ]]; then
    echo "Don't run the script as root. Instead use the user-account that will manage the cluster."
    exit 1
  fi
  if [[ "$#" -gt 1 ]]; then
    echo "Usage: $0 [<path to kubeadm config (kubeadm.yaml)>]" >&2
    exit 1
  fi

  local kubeadm_yaml=$1

  set -eu

  if [[ -z "${kubeadm_yaml}" ]] ; then
    kubeadm_yaml=$(create_default_kubeadm_config)
    trap "rm -f '${kubeadm_yaml}'" EXIT
  fi
  install_k8s_on_robot "${kubeadm_yaml}"
}

# If script is not being sourced, execute install_k8s_on_robot (else just define functions). This
# also protects the script against execution when the file is truncated.
#
# run as           |    $0     | $BASH_SOURCE
# -----------------+-----------+--------------
# bash script.sh   | script.sh | script.sh
# bash < script.sh | bash      |
# source script.sh | bash      | script.sh
#
# shellcheck disable=2128
# This code expects that $BASH_SOURCE expands to the first element.
if [[ "$0" == "$BASH_SOURCE" || -z "$BASH_SOURCE" ]] ; then
  main "$@"
fi
