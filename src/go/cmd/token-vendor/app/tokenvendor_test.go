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
		{"robot-dev-\ndevice", false},
		{"AAAAAAAAAAAAAAAAAAAarobot-dev-device-\neuwest1-test-com", false},
		{"TEST.com", false}, {"TEST.com.", false}, {"1-.test", false},
		// valid
		{"robot-dev-device", true}, {"1test", true}, {"1-test", true},
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

type serviceAccountNameTest struct {
	desc      string
	cfgSA     string
	reqSA     string
	wantSA    string
	wantError bool
}

func TestServiceAccountName(t *testing.T) {
	defaultSA := "robot-servcie@foo.iam.gserviceaccount.com"
	configuredSA := "custom@bar.iam.gserviceaccount.com"
	customSA := "unknowns@baz.iam.gserviceaccount.com"
	var cases = []serviceAccountNameTest{
		// happy cases
		{
			desc:   "nothing requested, nothign configured, gives default",
			wantSA: defaultSA,
		},
		{
			desc:   "nothing requested, sa configured, gives configured",
			cfgSA:  configuredSA,
			wantSA: configuredSA,
		},
		{
			desc:   "def requested, nothing configured, gives def",
			reqSA:  defaultSA,
			wantSA: defaultSA,
		},
		{
			desc:   "def requested, sa configured, gives def",
			cfgSA:  configuredSA,
			reqSA:  defaultSA,
			wantSA: defaultSA,
		},
		{
			desc:   "requested as configured, gives configured",
			cfgSA:  configuredSA,
			reqSA:  configuredSA,
			wantSA: configuredSA,
		},
		// error cases
		{
			desc:      "unknown requested, nothign configured, gives error",
			reqSA:     customSA,
			wantError: true,
		},
		{
			desc:      "unknown requested, sa configured, gives error",
			cfgSA:     configuredSA,
			reqSA:     customSA,
			wantError: true,
		},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			saName, err := serviceAccountName(defaultSA, test.cfgSA, test.reqSA)
			if (err == nil && test.wantError) || (err != nil && !test.wantError) {
				t.Fatalf("serviceAccountName(%q, %q, %q): got err %v, want %v",
					defaultSA, test.cfgSA, test.reqSA, err, test.wantError)
			} else if saName != test.wantSA {
				t.Fatalf("serviceAccountName(%q, %q, %q): got err %v, want %v",
					defaultSA, test.cfgSA, test.reqSA, err, test.wantError)
			}
		})
	}
}
