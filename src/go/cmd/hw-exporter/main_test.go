package main

import (
	"strings"
	"testing"

	"github.com/jaypipes/ghw"
	"github.com/jaypipes/ghw/pkg/option"
	"github.com/jaypipes/ghw/pkg/pci"
	"github.com/jaypipes/ghw/pkg/util"
	"github.com/jaypipes/pcidb"
	"github.com/prometheus/client_golang/prometheus/testutil"
)

var (
	testVendor = pcidb.Vendor{
		Name: "Intel Corporation",
		ID:   "8086",
	}
	unknownVendor = pcidb.Vendor{
		Name: util.UNKNOWN,
		ID:   "1234",
	}
	testProduct = pcidb.Product{
		Name: "Ethernet Connection (17) I219-LM",
		ID:   "1a1c",
	}
	unknownProduct = pcidb.Product{
		Name: util.UNKNOWN,
		ID:   "5678",
	}
	testClass = pcidb.Class{
		Name: "Ethernet controller",
		ID:   "0200",
	}
	unknownClass = pcidb.Class{
		Name: util.UNKNOWN,
		ID:   "9abc",
	}
)

func TestPciCollector_Collect(t *testing.T) {
	oldPCI := ghw.PCI
	t.Cleanup(func() {
		ghw.PCI = oldPCI
	})

	tests := []struct {
		desc    string
		devices []*pci.Device
		want    string
	}{{
		desc: "known devices",
		devices: []*pci.Device{{
			Vendor:  &testVendor,
			Product: &testProduct,
			Class:   &testClass,
			Driver:  "testdriver1",
		}, {
			Vendor:  &testVendor,
			Product: &testProduct,
			Class:   &testClass,
			Driver:  "testdriver2",
		}, {
			Vendor:  &testVendor,
			Product: &testProduct,
			Class:   &testClass,
			Driver:  "testdriver2",
		}},
		want: `
			# HELP pci_device_count Number of PCI devices by vendor, product, class, and driver.
			# TYPE pci_device_count gauge
			pci_device_count{class="Ethernet controller",driver="testdriver1",product="Ethernet Connection (17) I219-LM",vendor="Intel Corporation"} 1
			pci_device_count{class="Ethernet controller",driver="testdriver2",product="Ethernet Connection (17) I219-LM",vendor="Intel Corporation"} 2
		`,
	}, {
		desc: "unknown device",
		devices: []*pci.Device{{
			Vendor:  &unknownVendor,
			Product: &unknownProduct,
			Class:   &unknownClass,
			Driver:  "testdriver",
		}},
		want: `
			# HELP pci_device_count Number of PCI devices by vendor, product, class, and driver.
			# TYPE pci_device_count gauge
			pci_device_count{class="0x9abc",driver="testdriver",product="0x5678",vendor="0x1234"} 1
		`,
	}}

	collector := newPciCollector()
	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			ghw.PCI = func(opts ...*option.Option) (*pci.Info, error) {
				return &pci.Info{Devices: tc.devices}, nil
			}
			if err := testutil.CollectAndCompare(collector, strings.NewReader(tc.want), "pci_device_count"); err != nil {
				t.Errorf("unexpected collecting result:\n%s", err)
			}
		})
	}
}
