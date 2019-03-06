/*
 *
 * Copyright 2019 The Cloud Robotics Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>

#include "helloworld.grpc.pb.h"

using grpc::Channel;
using grpc::ChannelCredentials;
using grpc::ClientContext;
using grpc::Status;
using helloworld::Greeter;
using helloworld::HelloReply;
using helloworld::HelloRequest;

class GreeterClient {
 public:
  GreeterClient(std::shared_ptr<Channel> channel)
      : stub_(Greeter::NewStub(channel)) {}

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  std::string SayHello(const std::string& user) {
    // Data we are sending to the server.
    HelloRequest request;
    request.set_name(user);

    // Container for the data we expect from the server.
    HelloReply reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->SayHello(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      return reply.message();
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return "RPC failed";
    }
  }

 private:
  std::unique_ptr<Greeter::Stub> stub_;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    const std::string client_path(argv[0]);
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << client_path << " <address[:port]> [<name>]"
              << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << client_path
              << " www.endpoints.${PROJECT_ID}.cloud.goog:443" << std::endl;
    return 0;
  }

  // The first parameter is the server's address, optionally containing the
  // port.
  std::string grpc_endpoint(argv[1]);
  if (grpc_endpoint.find(":") == std::string::npos) {
    // Set the default port of the server.
    grpc_endpoint += ":50051";
  }

  // The optional second parameter is the name to be sent to the server.
  std::string name("world");
  if (argc >= 3) {
    name = argv[2];
  }

  std::cout << "Sending request to " << grpc_endpoint << " ..." << std::endl;

  // We are communicating via SSL to the endpoint service using the credentials
  // of the user or robot running the client.
  // We don't use credentials when connecting to localhost for testing.
  std::shared_ptr<ChannelCredentials> channel_creds;
  if (grpc_endpoint.find("localhost:") == 0 ||
      grpc_endpoint.find("127.0.0.1:") == 0) {
    channel_creds = grpc::InsecureChannelCredentials();
  } else {
    channel_creds = grpc::GoogleDefaultCredentials();
  }

  GreeterClient greeter(grpc::CreateChannel(grpc_endpoint, channel_creds));
  std::string user(name);
  std::string reply = greeter.SayHello(user);
  std::cout << "Greeter received: " << reply << std::endl;

  return 0;
}
