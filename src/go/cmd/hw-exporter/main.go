// hw-exporter exposes a Prometheus metric pci_device_count that indicates the
// number of each PCI device type (vendor/product/class/driver) installed on
// this node.
package main

import (
	"context"
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"syscall"

	"github.com/googlecloudrobotics/ilog"
	"github.com/jaypipes/ghw"
	"github.com/jaypipes/ghw/pkg/option"
	"github.com/jaypipes/ghw/pkg/util"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
)

var (
	metricsPort = flag.Int("metrics-port", 9999, "Port to expose Prometheus metrics on.")
	logLevel    = flag.Int("log-level", int(slog.LevelInfo), "the log message level required to be logged")
	chroot      = flag.String("chroot", "/", "Path to chroot into before collecting hardware info.")
)

type pciCollector struct {
	pciDeviceCount *prometheus.Desc
}

func newPciCollector() *pciCollector {
	return &pciCollector{
		pciDeviceCount: prometheus.NewDesc(
			"pci_device_count",
			"Number of PCI devices by vendor, product, class, and driver.",
			[]string{"vendor", "product", "class", "driver"},
			nil,
		),
	}
}

// Describe implements the prometheus.Collector interface.
func (c *pciCollector) Describe(ch chan<- *prometheus.Desc) {
	ch <- c.pciDeviceCount
}

// getNameOrID returns the name of a PCI device component, or its ID if the name is unknown.
func getNameOrID(name, id string) string {
	if name == util.UNKNOWN {
		return "0x" + id
	}
	return name
}

// Collect implements the prometheus.Collector interface.
func (c *pciCollector) Collect(ch chan<- prometheus.Metric) {
	pciInfo, err := ghw.PCI(&option.Option{Chroot: chroot})
	if err != nil {
		slog.Error("Failed to get PCI info", ilog.Err(err))
		return
	}

	deviceCounts := make(map[[4]string]float64)
	for _, device := range pciInfo.Devices {
		vendor := getNameOrID(device.Vendor.Name, device.Vendor.ID)
		product := getNameOrID(device.Product.Name, device.Product.ID)
		class := getNameOrID(device.Class.Name, device.Class.ID)
		labels := [4]string{vendor, product, class, device.Driver}
		deviceCounts[labels]++
	}

	for labels, count := range deviceCounts {
		ch <- prometheus.MustNewConstMetric(c.pciDeviceCount, prometheus.GaugeValue, count, labels[0], labels[1], labels[2], labels[3])
	}
}

func main() {
	flag.Parse()
	logHandler := ilog.NewLogHandler(slog.Level(*logLevel), os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	// Run once on startup to test container setup, this is useful during development.
	_, err := ghw.PCI(&option.Option{Chroot: chroot})
	if err != nil {
		slog.Error("Failed to get PCI info", ilog.Err(err))
		os.Exit(1)
	}

	// Construct and run the metrics server until stopped by k8s (or Ctrl+C).
	ctx, cancel := signal.NotifyContext(context.Background(), syscall.SIGTERM, syscall.SIGINT)
	defer cancel()

	registry := prometheus.NewRegistry()
	registry.MustRegister(newPciCollector())

	mux := http.NewServeMux()
	mux.Handle("/metrics", promhttp.HandlerFor(registry, promhttp.HandlerOpts{}))

	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", *metricsPort),
		Handler: mux,
	}

	go func() {
		slog.Info("Starting metrics server", slog.Int("port", *metricsPort))
		if err := server.ListenAndServe(); err != http.ErrServerClosed {
			slog.Error("Metrics server failed", ilog.Err(err))
			os.Exit(1)
		}
	}()

	// Call Shutdown() in the main goroutine because ListenAndServe() returns
	// immediately but if the main goroutine ends then, the process will stop
	// before finishing any ongoing requests.
	<-ctx.Done()
	slog.Info("Shutting down metrics server...")
	server.Shutdown(context.Background())
}
