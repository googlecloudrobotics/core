FROM grpc/cxx:1.12.0

WORKDIR /data

COPY server/server.cc ./server/
COPY proto/helloworld.proto ./proto/
COPY Makefile ./

RUN make greeter-server && make clean

CMD ["./greeter-server"]
