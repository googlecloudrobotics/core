FROM grpc/cxx:1.12.0

WORKDIR /data

COPY client/client.cc ./client/
COPY proto/helloworld.proto ./proto/
COPY Makefile ./

RUN make greeter-client && make clean

CMD ["./greeter-client"]
