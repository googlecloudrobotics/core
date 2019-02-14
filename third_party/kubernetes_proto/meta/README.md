# Manual modification of `generated.proto` #

The file `generated.proto` was modified from its original upstream version in
the following way:

All fields of type `Time` have been replaced with identically named fields of
type `google.protobuf.Timestamp`. This modification is necessary to enable
conversion between proto binary format and JSON without unmarshaling to Go
structs as an intermediate step.
