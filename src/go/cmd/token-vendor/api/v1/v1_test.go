package v1

import (
	"testing"
)

type isValidDeviceIDTest struct {
	deviceId string
	ret      bool
}

func TestValidateDeviceId(t *testing.T) {

	var isValidDeviceIDTests = []isValidDeviceIDTest{
		// invalid
		{"", false}, {`\\\\\\\\\\\\\`, false}, {"t\nt", false}, {"1", false},
		{"robot-dev-csieber-\neuwest1-c-googlers-com", false}, {"1test", false},
		{"AAAAAAAAAAAAAAAAAAAarobot-dev-csieber-\neuwest1-c-googlers-com", false},
		// valid
		{"robot-dev-csieber-euwest1-c-googlers.com", true}, {"TEST.com", true},
	}

	for _, test := range isValidDeviceIDTests {
		v := isValidDeviceID(test.deviceId)
		if v != test.ret {
			t.Errorf("isValidDeviceID(%q), got %v, want %v", test.deviceId, v, test.ret)
		}
	}
}
