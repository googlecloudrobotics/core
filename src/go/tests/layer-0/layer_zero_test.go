// layer_0 contains a test that brings up layer 0 (a single-node Kubernetes
// cluster) on a GCP VM. To run it on your workstation, first create a service
// account:
//   https://console.cloud.google.com/apis/credentials/serviceaccountkey
// make it a "Compute Admin", and download the JSON key. Go to:
//   https://console.cloud.google.com/iam-admin/serviceaccounts
// tick the box by ######-compute@, and make your new SA a "Service Account
// User". You might need to wait ~10 minutes for IAM to propagate. Then replace
// your-project-id below and run:
//   bazel test \
//     --test_env GCP_PROJECT_ID=your-project-id --test_env GCP_ZONE=europe-west1-c \
//     --test_env GOOGLE_APPLICATION_CREDENTIALS=path/to/credentials.json \
//     --test_tag_filters=external //src/go/tests/layer-0:all

package layer_0

import (
	"bytes"
	"context"
	"crypto/rand"
	"crypto/rsa"
	"crypto/x509"
	"encoding/pem"
	"fmt"
	"log"
	mrand "math/rand"
	"net"
	"os"
	"path"
	"strconv"
	"strings"
	"testing"
	"time"

	cssh "golang.org/x/crypto/ssh"
	"golang.org/x/oauth2/google"
	"google.golang.org/api/compute/v1"
)

const (
	alphanum = "abcdefghijklmnopqrstuvwxyz0123456789"

	// Note: to use a workspace-relative path, the go_test() must have `rundir = "."`.
	clusterInstallScript = "./src/bootstrap/robot/install_k8s_on_robot.sh"

	// Timeout when creating/deleting VMs.
	maxOperationTime = 10 * time.Minute

	// Timeout when attempting to establish an SSH connection.
	sshConnectAttemptTimeout = 5 * time.Second

	// How long to wait for SSH to come up on the host.
	sshConnectTimeout = 5 * time.Minute
)

var (
	GCP_PROJECT_ID = mustGetenv("GCP_PROJECT_ID")
	GCP_ZONE       = mustGetenv("GCP_ZONE")
)

func init() {
	mrand.Seed(time.Now().UnixNano())
}

func mustGetenv(key string) string {
	value := os.Getenv(key)
	if value == "" {
		log.Fatalf("ERROR: environment variable %s must be non-empty", key)
	}
	return value
}

func randomString(n int) string {
	b := make([]byte, n)
	for i := range b {
		b[i] = alphanum[mrand.Intn(len(alphanum))]
	}
	return string(b)
}

// generateKeypair generates an SSH keypair that will be used to connect to the VM.
func generateKeypair() ([]byte, []byte, error) {
	privateKey, err := rsa.GenerateKey(rand.Reader, 2048)
	if err != nil {
		return nil, nil, fmt.Errorf("failed to generate SSH private key: %v", err)
	}
	privatePEM := pem.EncodeToMemory(&pem.Block{
		Type:    "RSA PRIVATE KEY",
		Headers: nil,
		Bytes:   x509.MarshalPKCS1PrivateKey(privateKey),
	})
	publicKey, err := cssh.NewPublicKey(&privateKey.PublicKey)
	if err != nil {
		return nil, nil, fmt.Errorf("failed to generate SSH public key: %v", err)
	}
	publicPEM := cssh.MarshalAuthorizedKey(publicKey)
	return privatePEM, publicPEM, nil
}

// Get a service wrapper for the Compute Engine API using Application Default
// Credentials.
func getService() (*compute.Service, error) {
	ctx := context.Background()
	client, err := google.DefaultClient(ctx, compute.ComputeScope)
	if err != nil {
		return nil, err
	}
	return compute.New(client)
}

func waitForOp(service *compute.Service, project, zone, name string) error {
	for start := time.Now(); time.Since(start) < maxOperationTime; {
		op, err := service.ZoneOperations.Get(project, zone, name).Do()
		if err != nil {
			return err
		}
		if op.Status == "DONE" {
			if op.Error != nil {
				return fmt.Errorf("operation failed: %+v", op)
			}
			return nil
		}
		time.Sleep(time.Second)
	}
	return fmt.Errorf("operation timed out after %s: %s", maxOperationTime, name)
}

func createVM(privatePEM, publicPEM []byte, imageProject, imageFamily, name string) error {
	service, err := getService()
	if err != nil {
		return fmt.Errorf("get service: %v", err)
	}
	sshKeys := fmt.Sprintf("robot:%s\n", publicPEM)
	instance := &compute.Instance{
		Name: name,
		Disks: []*compute.AttachedDisk{
			{
				Type:       "PERSISTENT",
				Mode:       "READ_WRITE",
				Kind:       "compute#attachedDisk",
				Boot:       true,
				AutoDelete: true,
				InitializeParams: &compute.AttachedDiskInitializeParams{
					SourceImage: fmt.Sprintf("projects/%s/global/images/family/%s", imageProject, imageFamily),
					DiskSizeGb:  200,
					DiskType:    fmt.Sprintf("zones/%s/diskTypes/%s", GCP_ZONE, "pd-standard"),
				},
			},
		},
		MachineType: fmt.Sprintf("zones/%s/machineTypes/n1-standard-4", GCP_ZONE),
		Metadata: &compute.Metadata{
			Items: []*compute.MetadataItems{
				{
					Key:   "sshKeys",
					Value: &sshKeys,
				},
			},
		},
		NetworkInterfaces: []*compute.NetworkInterface{
			{
				AccessConfigs: []*compute.AccessConfig{
					{
						Name: "External NAT0",
						Type: "ONE_TO_ONE_NAT",
					},
				},
				Network:    "global/networks/default",
				Subnetwork: fmt.Sprintf("regions/%s/subnetworks/default", GCP_ZONE[:len(GCP_ZONE)-2]),
			},
		},
		ServiceAccounts: []*compute.ServiceAccount{
			{
				Email:  "default",
				Scopes: []string{compute.CloudPlatformScope},
			},
		},
		Tags: &compute.Tags{
			Items: []string{"delete-after-one-day"},
		},
	}

	op, err := service.Instances.Insert(GCP_PROJECT_ID, GCP_ZONE, instance).Do()
	if err != nil {
		return fmt.Errorf("insert: %v", err)
	}
	if err = waitForOp(service, GCP_PROJECT_ID, GCP_ZONE, op.Name); err != nil {
		return fmt.Errorf("wait for insert: %v", err)
	}

	return nil
}

func deleteVM(name string) error {
	service, err := getService()
	if err != nil {
		return err
	}
	op, err := service.Instances.Delete(GCP_PROJECT_ID, GCP_ZONE, name).Do()
	if err != nil {
		return err
	}
	return waitForOp(service, GCP_PROJECT_ID, GCP_ZONE, op.Name)
}

// isStartedByKokoro checks whether the test was started by Kokoro by checking
// the hostname. This is used to identify whether the VM's private IP should be
// used for SSH connections (valid from within the same cloud project as the
// target VM), or whether to use the public IP (valid from a dev workstation).
func isStartedByKokoro() (bool, error) {
	hostname, err := os.Hostname()
	if err != nil {
		return false, fmt.Errorf("failed to get hostname: %v", err)
	}
	return strings.HasPrefix(hostname, "kokoro"), nil
}

// getSSH gets a connected SSH client, connecting to the public or private IP
// depending on how the test was started.
func getSSH(t *testing.T, name string, privatePEM []byte) (*cssh.Client, error) {
	service, err := getService()
	if err != nil {
		return nil, err
	}
	instance, err := service.Instances.Get(GCP_PROJECT_ID, GCP_ZONE, name).Do()
	if err != nil {
		return nil, err
	}
	var addr string
	if onKokoro, err := isStartedByKokoro(); err != nil {
		return nil, err
	} else if onKokoro {
		addr = instance.NetworkInterfaces[0].NetworkIP + ":22"
		t.Logf("Detected that VM is started by Kokoro, so using private IP: %v", addr)
	} else {
		addr = instance.NetworkInterfaces[0].AccessConfigs[0].NatIP + ":22"
		t.Logf("Using public VM IP: %v", addr)
	}

	signer, err := cssh.ParsePrivateKey(privatePEM)
	if err != nil {
		return nil, err
	}
	config := &cssh.ClientConfig{
		User: "robot",
		Auth: []cssh.AuthMethod{
			cssh.PublicKeys(signer),
		},
		HostKeyCallback: cssh.InsecureIgnoreHostKey(),
	}

	var retryErr error
	for start := time.Now(); time.Since(start) < sshConnectTimeout; {
		conn, err := net.DialTimeout("tcp", addr, sshConnectAttemptTimeout)
		if err != nil {
			retryErr = err
			continue
		}
		c, chans, reqs, err := cssh.NewClientConn(conn, addr, config)
		if err != nil {
			retryErr = err
			continue
		}
		return cssh.NewClient(c, chans, reqs), nil
	}
	return nil, fmt.Errorf("timeout for %s: %v", addr, retryErr)
}

// uploadLocalFile uploads a local file over SSH.
func uploadLocalFile(client *cssh.Client, localPath string, remotePath string, mode uint32) error {
	f, err := os.Open(localPath)
	if err != nil {
		return fmt.Errorf("failed to open %s: %v", localPath, err)
	}
	session, err := client.NewSession()
	if err != nil {
		return fmt.Errorf("new session: %v", err)
	}
	defer session.Close()
	session.Stdin = f
	if output, err := session.CombinedOutput(fmt.Sprintf("cat > %q", remotePath)); err != nil {
		return fmt.Errorf("upload file: %v, %s", err, output)
	}
	if output, err := runCommand(client, fmt.Sprintf("chmod %o %q 2>&1", mode, remotePath)); err != nil {
		return fmt.Errorf("chmod: %v, %s", err, output)
	}
	return nil
}

// To avoid mixing the output streams of multiple commands running in
// parallel tests, we combine stdout and stderr on the remote host, and
// capture stdout and write it to the test log here.
func runCommand(client *cssh.Client, command string) ([]byte, error) {
	session, err := client.NewSession()
	if err != nil {
		return nil, err
	}
	defer session.Close()
	return session.Output(command + " 2>&1")
}

// testCommand runs a command that is expected to succeed. If it fails, it
// logs the command's output and returns an error.
func testCommand(t *testing.T, client *cssh.Client, command string) error {
	if stdout, err := runCommand(client, command+" 2>&1"); err != nil {
		t.Logf("`%s` failed with %v:\n%s", command, err, stdout)
		return err
	}
	return nil
}

// logCommand runs a command that could be helpful for debugging. The command's
// output is written to the test log.
func logCommand(t *testing.T, client *cssh.Client, command string) {
	stdout, err := runCommand(client, command+" 2>&1")
	t.Logf("+ %s\n%s", command, stdout)
	if err != nil {
		t.Logf("`%s` failed with %v:\n", command, err)
	}
}

// uploadAndRun uploads a local bash script over SSH and runs it.
func uploadAndRun(t *testing.T, client *cssh.Client, localPath string, args ...string) error {
	remotePath := "./" + path.Base(localPath)
	if err := uploadLocalFile(client, localPath, remotePath, 0755); err != nil {
		return err
	}
	quotedArgs := make([]string, len(args))
	for i, arg := range args {
		quotedArgs[i] = strconv.Quote(arg)
	}
	command := fmt.Sprintf(`bash -x %s %s`, remotePath, strings.Join(quotedArgs, " "))
	if err := testCommand(t, client, command); err != nil {
		return fmt.Errorf("failed to run `%s` on VM: %v", remotePath, err)
	}
	return nil
}

// testClusterNetworking checks that the internet is reachable from the cluster.
// It does this with a K8s Job that uses busybox to ping google.com.
func testClusterNetworking(t *testing.T, client *cssh.Client) error {
	session, err := client.NewSession()
	if err != nil {
		return err
	}
	defer session.Close()
	session.Stdin = bytes.NewBufferString(`
apiVersion: batch/v1
kind: Job
metadata:
  name: ping
spec:
  template:
    spec:
      containers:
        - name: ping
          image: busybox
          command: ["ping",  "-c1", "google.com"]
      restartPolicy: OnFailure
`)
	if stdout, err := session.Output("kubectl create -f - 2>&1"); err != nil {
		t.Logf("`kubectl create -f -` failed with %v:\n%s", err, stdout)
		return fmt.Errorf("failed to create job on VM: %v", err)
	}
	waitCommand := "kubectl wait --for condition=Complete job/ping --timeout=5m"
	if err := testCommand(t, client, waitCommand); err != nil {
		t.Error("Job `ping` failed to complete on VM")
		// Dump pod logs to help debug failure.
		logCommand(t, client, "kubectl logs -l job-name=ping")
	}
	return nil
}

// TestInstallLayerZero creates a new VM and installs a Kubernetes cluster on it.
func TestInstallLayerZero(t *testing.T) {
	privatePEM, publicPEM, err := generateKeypair()
	if err != nil {
		t.Fatal(err)
	}

	// Together, these identify the public image used for the VM, without
	// pinning a specific version. We test each image in parallel as the tests
	// are independent and take 3-4 minutes.
	testCases := []struct {
		imageProject string
		imageFamily  string
	}{
		{"ubuntu-os-cloud", "ubuntu-1604-lts"},
		{"ubuntu-os-cloud", "ubuntu-1804-lts"},
	}
	for _, tc := range testCases {
		// Make the closure capture tc by value rather than by reference.
		tc := tc
		t.Run(tc.imageFamily, func(t *testing.T) {
			t.Parallel()
			name := "layer-zero-test-vm-" + randomString(8)
			defer func() {
				t.Log("Deleting VM...")
				if err := deleteVM(name); err != nil {
					t.Error("Failed to delete VM:", err)
				}
			}()

			t.Logf("Creating VM...")
			if createVM(privatePEM, publicPEM, tc.imageProject, tc.imageFamily, name) != nil {
				t.Fatal(err)
			}
			t.Log("Establishing SSH connection to VM...")
			client, err := getSSH(t, name, privatePEM)
			if err != nil {
				t.Fatal("Failed to SSH to VM:", err)
			}

			t.Log("Installing cluster on VM...")
			if err := uploadAndRun(t, client, clusterInstallScript); err != nil {
				t.Fatal("Failed to install cluster:", err)
			}

			t.Log("Testing networking on VM cluster...")
			if err := testClusterNetworking(t, client); err != nil {
				t.Fatal("Failed networking test:", err)
			}
		})
	}
}
