package com.cloudrobotics.hello_world_client;

import cloudrobotics.hello_world.v1alpha1.K8sHelloWorldGrpc;
import cloudrobotics.hello_world.v1alpha1.Service;
import io.grpc.ManagedChannelBuilder;
import java.util.logging.Logger;

/** */
final class Main {

  private static final Logger logger = Logger.getLogger(Main.class.getName());

  public static void main(String[] args) {
    K8sHelloWorldGrpc.K8sHelloWorldBlockingStub stub =
        K8sHelloWorldGrpc.newBlockingStub(
            ManagedChannelBuilder.forAddress("localhost", 50051).usePlaintext().build());
    Service.HelloWorld world =
        stub.get(Service.GetHelloWorldRequest.newBuilder().setName("foo").build());
    System.out.println(world.getMetadata().getResourceVersion());
  }
}
