// Copyright 2025 The Cloud Robotics Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"log"
	"net"

	"github.com/google/nftables"
	"github.com/google/nftables/expr"
)

var metadataRule = []byte("metadata-nat")

func addNATRule() error {
	con, err := nftables.New()
	if err != nil {
		return fmt.Errorf("nftables new: %v", err)
	}
	table := con.AddTable(&nftables.Table{
		Name:   "nat",
		Family: nftables.TableFamilyIPv4,
	})
	accept := nftables.ChainPolicyAccept
	prerouting := con.AddChain(&nftables.Chain{
		Name:     "PREROUTING",
		Table:    table,
		Hooknum:  nftables.ChainHookPrerouting,
		Priority: nftables.ChainPriorityNATDest,
		Type:     nftables.ChainTypeNAT,
		Policy:   &accept,
	})
	destinationPort := make([]byte, 2)
	binary.BigEndian.PutUint16(destinationPort, uint16(*port))
	destinationIP := net.ParseIP(*bindIP)
	if destinationIP == nil {
		return fmt.Errorf("%s is not a valid IPv4 address", *bindIP)
	}
	e := []expr.Any{
		// Load network IP into register 1
		&expr.Payload{
			OperationType:  expr.PayloadLoad,
			DestRegister:   1,
			SourceRegister: 0,
			Base:           expr.PayloadBaseNetworkHeader,
			Offset:         16,
			Len:            4,
			CsumType:       expr.CsumTypeNone,
			//CsumOffset:     0,
			//CsumFlags:      0,
		},
		// Match register 1 with metadata server IP
		&expr.Cmp{
			Op:       expr.CmpOpEq,
			Register: 1,
			Data:     net.IPv4(169, 254, 169, 254).To4(),
		},
		// Load transport layer protocol into register 1
		&expr.Meta{
			Key:            expr.MetaKeyL4PROTO,
			SourceRegister: false,
			Register:       1,
		},
		// Match transport layer protocol with TCP
		&expr.Cmp{
			Op:       expr.CmpOpEq,
			Register: 1,
			Data:     []byte{6}, // TCP
		},
		// Load network port into register 1
		&expr.Payload{
			OperationType:  expr.PayloadLoad,
			DestRegister:   1,
			SourceRegister: 0,
			Base:           expr.PayloadBaseTransportHeader,
			Offset:         2,
			Len:            2,
			CsumType:       expr.CsumTypeNone,
			//CsumOffset:     0,
			//CsumFlags:      0,
		},
		// Match register 1 with port 80
		&expr.Cmp{
			Op:       expr.CmpOpEq,
			Register: 1,
			Data:     []byte{0, 80},
		},
		// Adding a counter helps debugging
		&expr.Counter{
			Bytes:   0,
			Packets: 0,
		},
		// Place destination IP in register 1
		&expr.Immediate{
			Register: 1,
			Data:     net.IPv4(127, 0, 0, 1).To4(),
		},
		// Place destination port in register 2
		&expr.Immediate{
			Register: 2,
			Data:     destinationPort,
		},
		&expr.NAT{
			Type:        1,
			Family:      2,
			RegAddrMin:  1,
			RegAddrMax:  1,
			RegProtoMin: 2,
			RegProtoMax: 2,
			Random:      false,
			FullyRandom: false,
			Persistent:  false,
		},
	}

	con.AddRule(&nftables.Rule{
		Table: table,
		Chain: prerouting,
		// Flags:
		Exprs:    e,
		UserData: metadataRule,
	})

	if err := con.Flush(); err != nil {
		return fmt.Errorf("nftables flush: %v", err)
	}
	return nil
}

func removeNATRule() {
	con, err := nftables.New()
	if err != nil {
		log.Printf("Warning: nftables invocation failed: %v", err)
	}
	table := con.AddTable(&nftables.Table{
		Name:   "nat",
		Family: nftables.TableFamilyIPv4,
	})
	accept := nftables.ChainPolicyAccept
	prerouting := con.AddChain(&nftables.Chain{
		Name:     "PREROUTING",
		Table:    table,
		Hooknum:  nftables.ChainHookPrerouting,
		Priority: nftables.ChainPriorityNATDest,
		Type:     nftables.ChainTypeNAT,
		Policy:   &accept,
	})
	rs, err := con.GetRules(table, prerouting)
	if err != nil {
		log.Printf("Warning: nftables invocation failed: %v", err)
	}
	for _, r := range rs {
		if bytes.Equal(r.UserData, metadataRule) {
			con.DelRule(r)
		}
	}
	if err := con.Flush(); err != nil {
		log.Printf("Warning: nftables invocation failed: %v", err)
	}
}
