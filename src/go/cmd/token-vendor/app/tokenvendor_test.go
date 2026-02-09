package app

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"reflect"
	"testing"
	"time"

	"github.com/form3tech-oss/jwt-go"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/memory"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/tokensource"
)

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

type keyOptionsTest struct {
	desc      string
	opts      repository.KeyOptions
	wantError bool
}

func TestKeyOptions(t *testing.T) {
	var cases = []keyOptionsTest{
		{
			desc: "empty opts are good",
			opts: repository.KeyOptions{},
		},
		{
			desc: "one good email, one empty",
			opts: repository.KeyOptions{ServiceAccount: "svc@example.com"},
		},
		{
			desc: "two good emails",
			opts: repository.KeyOptions{ServiceAccount: "svc@example.com", ServiceAccountDelegate: "del@example.com"},
		},
		{
			desc:      "bad email #1",
			opts:      repository.KeyOptions{ServiceAccount: "svc"},
			wantError: true,
		},
		{
			desc:      "bad email #2",
			opts:      repository.KeyOptions{ServiceAccount: "svc@"},
			wantError: true,
		},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			err := validateKeyOptions(test.opts)
			haveError := err != nil
			if haveError != test.wantError {
				if test.wantError {
					t.Fatalf("validateKeyOptions returned %v, but wanted error", err)
				} else {
					t.Fatalf("validateKeyOptions returned %v, but wanted no error", err)
				}
			}
		})
	}
}

func TestTokenVendor_ValidateJWT(t *testing.T) {
	type fields struct {
		repo          repository.PubKeyRepository
		v             *oauth.TokenVerifier
		ts            *tokensource.GCPTokenSource
		accAud        string
		defaultSAName string
	}

	const deviceId = "test-device-id"
	devicePubKey := &repository.Key{
		PublicKey: "test-device-public-key",
	}

	defaultFields := fields{
		repo:          getInMemoryRepo(deviceId, devicePubKey.PublicKey),
		v:             nil,
		ts:            nil,
		accAud:        "",
		defaultSAName: "robot-service@testing.iam.gserviceaccount.com",
	}

	type args struct {
		jwtk string
	}
	tests := []struct {
		name    string
		fields  fields
		args    args
		want    *DeviceAuth
		wantErr bool
	}{
		{
			name:   "sa-success",
			fields: defaultFields,
			args: args{
				jwtk: createFakeJWTWithSubject(t, deviceId, createSAFromName("robot-service")),
			},
			want: &DeviceAuth{
				DeviceID:   deviceId,
				Key:        devicePubKey,
				ServiceAcc: createSAFromName("robot-service"),
			},
			wantErr: false,
		},
		{
			name:   "jwt-subject-empty",
			fields: defaultFields,
			args: args{
				jwtk: createFakeJWTWithSubject(t, deviceId, ""),
			},
			want: &DeviceAuth{
				DeviceID:   deviceId,
				Key:        devicePubKey,
				ServiceAcc: "",
			},
			wantErr: false,
		},
		{
			name:   "jwt-subject-not-an-SA",
			fields: defaultFields,
			args: args{
				jwtk: createFakeJWTWithSubject(t, deviceId, "robot-rock"),
			},
			want: &DeviceAuth{
				DeviceID:   deviceId,
				Key:        devicePubKey,
				ServiceAcc: "",
			},
			wantErr: false,
		},
		{
			name:   "jwt-subject-not-gcp-SA",
			fields: defaultFields,
			args: args{
				jwtk: createFakeJWTWithSubject(t, deviceId, "robot-rock@iam.gserviceaccount.com"),
			},
			want: &DeviceAuth{
				DeviceID:   deviceId,
				Key:        devicePubKey,
				ServiceAcc: "",
			},
			wantErr: false,
		},
		{
			name:   "jwt-subject-shall-not-pass",
			fields: defaultFields,
			args: args{
				jwtk: createFakeJWTWithSubject(t, deviceId, "robot-rock@.iam.gserviceaccount.com"),
			},
			want: &DeviceAuth{
				DeviceID:   deviceId,
				Key:        devicePubKey,
				ServiceAcc: "",
			},
			wantErr: false,
		},
		{
			name:   "jwt-subject-not-really-SA",
			fields: defaultFields,
			args: args{
				jwtk: createFakeJWTWithSubject(t, deviceId, "arn:aws:iam::123456789012:role/my-role"),
			},
			want: &DeviceAuth{
				DeviceID:   deviceId,
				Key:        devicePubKey,
				ServiceAcc: "",
			},
			wantErr: false,
		},
	}
	jwtVerifySignature = func(jwtk string, pubKey string) error {
		// Tests do not validate signature
		return nil
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			tv := &TokenVendor{
				repo:          tt.fields.repo,
				v:             tt.fields.v,
				ts:            tt.fields.ts,
				accAud:        tt.fields.accAud,
				defaultSAName: tt.fields.defaultSAName,
			}
			got, err := tv.ValidateJWT(context.Background(), tt.args.jwtk)
			if (err != nil) != tt.wantErr {
				t.Errorf("ValidateJWT() error = %v, wantErr %v", err, tt.wantErr)
				return
			}
			if !reflect.DeepEqual(got, tt.want) {
				t.Errorf("ValidateJWT() got = %v, want %v", got, tt.want)
			}
		})
	}
}

func getInMemoryRepo(deviceId, key string) repository.PubKeyRepository {
	ctx := context.Background()
	repo, _ := memory.NewMemoryRepository(ctx)
	_ = repo.PublishKey(ctx, deviceId, key)
	return repo
}

func createFakeJWTWithSubject(t *testing.T, deviceId, subject string) string {
	jwtWithSubject := jwt.StandardClaims{
		Audience:  nil,
		ExpiresAt: time.Now().Add(10 * time.Second).Unix(),
		Id:        t.Name(),
		IssuedAt:  0,
		Issuer:    deviceId,
		NotBefore: 0,
		Subject:   subject,
	}

	buffer := new(bytes.Buffer)
	if err := json.NewEncoder(buffer).Encode(&jwtWithSubject); err != nil {
		t.Errorf("failed to serialize jwt: %v", err)
	}

	return jwt.EncodeSegment([]byte(`{"typ": "JWT"}`)) +
		"." + jwt.EncodeSegment(buffer.Bytes()) +
		"." + jwt.EncodeSegment([]byte("no-signature-present"))
}

func createSAFromName(name string) string {
	return fmt.Sprintf("%s@%s.iam.gserviceaccount.com", name, "test-project")
}
