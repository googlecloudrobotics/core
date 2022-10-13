package main

import (
	"context"
	"time"

	"github.com/pkg/errors"
	log "github.com/sirupsen/logrus"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/cloudiot"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/k8s"
)

// runMigration migrates public keys from Cloud IoT to Kubernetes backend.
func runMigration() {
	log.SetLevel(log.DebugLevel)
	log.Info("running migration of public keys from Cloud IoT to Kubernetes backend.")
	ctx := context.Background()
	r := cloudiot.Registry{Project: *project, Region: *region, Registry: *registry}
	iotR, err := cloudiot.NewCloudIoTRepository(ctx, r, nil)
	if err != nil {
		log.Panic(err)
	}
	config, err := rest.InClusterConfig()
	if err != nil {
		log.Panic(err)
	}
	cs, err := kubernetes.NewForConfig(config)
	if err != nil {
		log.Panic(err)
	}
	k8sR, err := k8s.NewK8sRepository(ctx, cs, *namespace)
	if err != nil {
		log.Panic(err)
	}
	if _, err = migrate(ctx, iotR, k8sR); err != nil {
		log.Panic(err)
	}
}

type summary struct {
	iotCntBefore int           // devices found on IoT before (including blocked ones)
	k8sCntBefore int           // devices found on K8s before (including blocked ones)
	migrated     int           // devices migrated to K8s
	existed      int           // already on K8s before migration
	blocked      int           // devices blocked or have no key on IoT and thus not migrated
	k8sOwned     int           // new on K8s, not found on Cloud IoT
	elapsed      time.Duration // duration of whole migration
}

// migrate copies device keys from the Cloud IoT to the Kubernetes backend.
//
// Any error stops the whole migration. The whole operation should be
// idempotent and can run again and at any time.
// We can not create blocked devices on Kubernetes. Thus we can not determine by
// key count if we already migrated all keys from Cloud IoT and have to check
// every device.
func migrate(ctx context.Context, i *cloudiot.CloudIoTRepository, k *k8s.K8sRepository) (*summary, error) {
	s := summary{}
	start := time.Now()
	lsIoT, err := i.ListAllDeviceIDs(ctx)
	if err != nil {
		return nil, errors.Wrap(err, "failed to list IoT device identifiers")
	}
	s.iotCntBefore = len(lsIoT)
	log.Infof("found %d keys on Cloud IoT.", s.iotCntBefore)
	lsK8s, err := k.ListAllDeviceIDs(ctx)
	if err != nil {
		return nil, errors.Wrap(err, "failed to list Kubernetes device identifiers")
	}
	s.k8sCntBefore = len(lsK8s)
	log.Infof("found %d keys on Kubernetes.", s.k8sCntBefore)
	// calculated intersection
	m := make(map[string]struct{})
	for _, id := range lsIoT {
		m[id] = struct{}{}
	}
	for _, id := range lsK8s {
		if _, found := m[id]; found {
			s.existed += 1
			delete(m, id)
		} else {
			s.k8sOwned += 1
		}
	}
	if len(m) == 0 {
		log.Info("no new keys found on Cloud IoT, nothing to to do")
		return &s, nil
	}
	// run migration
	log.Infof("found %d keys in Cloud IoT which are not on Kubernetes, migrating", len(m))
	for id := range m {
		log.Infof("migrating device %q", id)
		pubKey, err := i.LookupKey(ctx, id)
		if err != nil {
			return nil, errors.Wrapf(err, "failed to lookup key %q on Cloud IoT", id)
		}
		if pubKey == "" {
			log.Infof("device %q is blocked or for some reason has no public key, not migrating it", id)
			s.blocked += 1
			continue
		}
		if err = k.PublishKey(ctx, id, pubKey); err != nil {
			return nil, errors.Wrapf(err, "failed to publish key for %q to Kubernetes", id)
		}
		s.migrated += 1
	}
	elapsed := time.Since(start)
	log.Infof("Summary: IoT devices %d, K8s devices %d, already existed on K8s %d, on K8s but not on IoT %d, migrated now %d, blocked %d, elapsed %s",
		s.iotCntBefore, s.k8sCntBefore, s.existed, s.k8sOwned, s.migrated, s.blocked, elapsed)
	return &s, nil
}
