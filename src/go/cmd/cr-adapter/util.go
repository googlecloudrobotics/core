// Copyright 2019 The Cloud Robotics Authors
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
	"net/http"

	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

var k8sStatusTable = map[metav1.StatusReason]codes.Code{
	metav1.StatusReasonUnknown:              codes.Unknown,
	metav1.StatusReasonUnauthorized:         codes.Unauthenticated,
	metav1.StatusReasonForbidden:            codes.PermissionDenied,
	metav1.StatusReasonNotFound:             codes.NotFound,
	metav1.StatusReasonAlreadyExists:        codes.AlreadyExists,
	metav1.StatusReasonConflict:             codes.Aborted,
	metav1.StatusReasonGone:                 codes.NotFound,
	metav1.StatusReasonInvalid:              codes.InvalidArgument,
	metav1.StatusReasonServerTimeout:        codes.Unavailable,
	metav1.StatusReasonTimeout:              codes.Unavailable,
	metav1.StatusReasonTooManyRequests:      codes.ResourceExhausted,
	metav1.StatusReasonBadRequest:           codes.InvalidArgument,
	metav1.StatusReasonMethodNotAllowed:     codes.InvalidArgument,
	metav1.StatusReasonNotAcceptable:        codes.InvalidArgument,
	metav1.StatusReasonUnsupportedMediaType: codes.InvalidArgument,
	metav1.StatusReasonInternalError:        codes.Internal,
	metav1.StatusReasonExpired:              codes.OutOfRange,
	metav1.StatusReasonServiceUnavailable:   codes.Unavailable,
}

var httpStatusTable = map[int]codes.Code{
	http.StatusBadRequest:         codes.Internal,
	http.StatusUnauthorized:       codes.Unauthenticated,
	http.StatusForbidden:          codes.PermissionDenied,
	http.StatusNotFound:           codes.Unimplemented,
	http.StatusTooManyRequests:    codes.Unavailable,
	http.StatusBadGateway:         codes.Unavailable,
	http.StatusServiceUnavailable: codes.Unavailable,
	http.StatusGatewayTimeout:     codes.Unavailable,
}

func k8sStatusToGRPCStatus(in metav1.Status) *status.Status {
	code, ok := k8sStatusTable[in.Reason]
	if !ok {
		code, ok = httpStatusTable[int(in.Code)]
		if !ok {
			code = codes.Unknown
		}
	}

	return status.New(code, in.Message)
}

func k8sErrorToGRPCError(err error) error {
	// Try reading the Kubernetes-generated error that DoRaw()
	// auto-generated from the response code.
	statusError, ok := err.(*errors.StatusError)
	if ok {
		return k8sStatusToGRPCStatus(statusError.ErrStatus).Err()
	}
	return status.Errorf(codes.Unknown, err.Error())
}
