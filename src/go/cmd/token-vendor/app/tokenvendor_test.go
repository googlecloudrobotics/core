package app

import "testing"

type isValidDeviceIDTest struct {
	deviceId string
	ret      bool
}

func TestValidateDeviceId(t *testing.T) {

	var isValidDeviceIDTests = []isValidDeviceIDTest{
		// invalid
		{"", false}, {`\\\\\\\\\\\\\`, false}, {"t\nt", false}, {"1", false},
		{"robot-dev-\ndevice", false}, {"1test", false},
		{"AAAAAAAAAAAAAAAAAAAarobot-dev-device-\neuwest1-test-com", false},
		// valid
		{"robot-dev-device", true}, {"TEST.com", true},
	}

	for _, test := range isValidDeviceIDTests {
		v := IsValidDeviceID(test.deviceId)
		if v != test.ret {
			t.Errorf("isValidDeviceID(%q), got %v, want %v", test.deviceId, v, test.ret)
		}
	}
}

type acceptedAudienceTest struct {
	desc         string
	aud          string
	accAud       string
	wantAccepted bool
}

func TestAcceptedAudience(t *testing.T) {
	var cases = []acceptedAudienceTest{
		// accepted audiences
		{"URL exact", "http://something/", "http://something/", true},
		{"URL one of them", "http://anything/ http://something/", "http://something/", true},
		{"URL with parameters", "http://something/?token_type=access_token", "http://something/", true},
		// not accepted audiences
		{"empty", "", "http://something/", false},
		{"some other URL", "http://something/else", "http://something/", false},
		{"URL one of them, but concat", "http://anything/http://something/", "http://something/", false},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			err := acceptedAudience(test.aud, test.accAud)
			if (err == nil && !test.wantAccepted) || (err != nil && test.wantAccepted) {
				t.Fatalf("acceptedAudience(%q, %q): got err %v, want %v",
					test.aud, test.accAud, err, test.wantAccepted)
			}
		})
	}
}
