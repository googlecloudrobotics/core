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

package com.cloudrobotics.map;

import static com.cloudrobotics.map.v1alpha1.Layer.LayerType.OCCUPANCY_GRID_LAYER;
import static com.cloudrobotics.map.v1alpha1.Layer.LayerType.POINT_LAYER;
import static org.hamcrest.Matchers.containsInAnyOrder;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.fail;

import com.cloudrobotics.common.v1alpha1.Rigid2d;
import com.cloudrobotics.common.v1alpha1.Vector2d;
import com.cloudrobotics.framework.DatastoreTestRule;
import com.cloudrobotics.map.v1alpha1.DeleteLayerRequest;
import com.cloudrobotics.map.v1alpha1.DeleteMapRequest;
import com.cloudrobotics.map.v1alpha1.GetLayerRequest;
import com.cloudrobotics.map.v1alpha1.GetMapRequest;
import com.cloudrobotics.map.v1alpha1.GetOccupancyGridRequest;
import com.cloudrobotics.map.v1alpha1.GetPointRequest;
import com.cloudrobotics.map.v1alpha1.Layer;
import com.cloudrobotics.map.v1alpha1.Layer.LayerType;
import com.cloudrobotics.map.v1alpha1.ListLayersRequest;
import com.cloudrobotics.map.v1alpha1.ListPointsRequest;
import com.cloudrobotics.map.v1alpha1.Map;
import com.cloudrobotics.map.v1alpha1.MapServiceGrpc;
import com.cloudrobotics.map.v1alpha1.OccupancyGrid;
import com.cloudrobotics.map.v1alpha1.Point;
import com.google.common.flogger.FluentLogger;
import com.google.protobuf.Empty;
import io.grpc.Status.Code;
import io.grpc.StatusRuntimeException;
import io.grpc.testing.GrpcServerRule;
import org.junit.Before;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class MapServiceTest {
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  @ClassRule public static DatastoreTestRule datastoreRule = new DatastoreTestRule();
  @Rule public final GrpcServerRule grpcServerRule = new GrpcServerRule().directExecutor();

  private MapServiceGrpc.MapServiceBlockingStub stub;

  @Before
  public void setUp() {
    grpcServerRule.getServiceRegistry().addService(new MapService(datastoreRule.getDatastore()));
    stub = MapServiceGrpc.newBlockingStub(grpcServerRule.getChannel());
  }

  @Test
  public void testCreateMap_hasId() {
    Map response = stub.createMap(Map.newBuilder().setDisplayName("Test").build());
    logger.atInfo().log("CreateMapResponse: %s", response);
    assertNotEquals(0, response.getMapId());
  }

  @Test
  public void testGetMap_returnsMap() {
    Map response = stub.createMap(Map.newBuilder().setDisplayName("Test").build());
    Map received = stub.getMap(GetMapRequest.newBuilder().setMapId(response.getMapId()).build());
    assertEquals(response, received);
  }

  @Test
  public void testGetMap_nonExistingId_throwsNotFound() {
    try {
      stub.getMap(GetMapRequest.newBuilder().setMapId(123).build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }

  @Test
  public void testListMaps_containsElements() {
    Map map1 = stub.createMap(Map.newBuilder().setDisplayName("map1").build());
    Map map2 = stub.createMap(Map.newBuilder().setDisplayName("map2").build());

    stub.getMap(GetMapRequest.newBuilder().setMapId(map1.getMapId()).buildPartial());
    stub.getMap(GetMapRequest.newBuilder().setMapId(map2.getMapId()).buildPartial());

    assertThat(
        stub.listMaps(Empty.getDefaultInstance()).getMapsList(), containsInAnyOrder(map1, map2));
  }

  @Test
  public void testGetMap_afterDelete_throwsNotFound() {
    Map map = stub.createMap(Map.newBuilder().setDisplayName("Test").build());
    stub.deleteMap(DeleteMapRequest.newBuilder().setMapId(map.getMapId()).build());
    try {
      stub.getMap(GetMapRequest.newBuilder().setMapId(map.getMapId()).build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }

  @Test
  public void testCreateLayer_hasId() {
    Map createMapResponse = stub.createMap(Map.newBuilder().setDisplayName("TestMap").build());
    Layer createLayerResponse =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(createMapResponse.getMapId())
                .setDisplayName("TestLayer")
                .setType(LayerType.OCCUPANCY_GRID_LAYER)
                .build());
    assertNotEquals(0, createLayerResponse.getLayerId());
  }

  @Test
  public void testGetLayer_returnsLayer() {
    Map createMapResponse = stub.createMap(Map.newBuilder().setDisplayName("TestMap").build());
    Layer createLayerResponse =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(createMapResponse.getMapId())
                .setDisplayName("TestLayer")
                .setType(LayerType.OCCUPANCY_GRID_LAYER)
                .build());
    Layer received =
        stub.getLayer(
            GetLayerRequest.newBuilder()
                .setMapId(createMapResponse.getMapId())
                .setLayerId(createLayerResponse.getLayerId())
                .build());
    assertEquals(received, createLayerResponse);
  }

  @Test
  public void testGetLayer_nonExistingId_throwsNotFound() {
    try {
      stub.getLayer(GetLayerRequest.newBuilder().setMapId(123).setLayerId(456).build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }

  @Test
  public void testGetLayer_wrongMapId_throwsNotFound() {
    try {
      Map createMapResponse = stub.createMap(Map.newBuilder().setDisplayName("TestMap").build());
      Layer createLayerResponse =
          stub.createLayer(
              Layer.newBuilder()
                  .setMapId(createMapResponse.getMapId())
                  .setDisplayName("TestLayer")
                  .setType(LayerType.OCCUPANCY_GRID_LAYER)
                  .build());
      stub.getLayer(
          GetLayerRequest.newBuilder()
              .setMapId(123)
              .setLayerId(createLayerResponse.getLayerId())
              .build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }

  @Test
  public void testListLayers_containsElements() {
    Map map1 = stub.createMap(Map.newBuilder().setDisplayName("map1").build());
    Map map2 = stub.createMap(Map.newBuilder().setDisplayName("map2").build());
    Layer layer1 =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map1.getMapId())
                .setDisplayName("layer1")
                .setType(LayerType.OCCUPANCY_GRID_LAYER)
                .build());
    Layer layer2 =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map1.getMapId())
                .setDisplayName("layer2")
                .setType(LayerType.OCCUPANCY_GRID_LAYER)
                .build());
    Layer layer3 =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map2.getMapId())
                .setDisplayName("layer3")
                .setType(LayerType.OCCUPANCY_GRID_LAYER)
                .build());
    assertThat(
        stub.listLayers(ListLayersRequest.newBuilder().setMapId(map1.getMapId()).build())
            .getLayersList(),
        containsInAnyOrder(layer1, layer2));
  }

  @Test
  public void testGetLayer_afterDeleteLayer_throwsNotFound() {
    Map map = stub.createMap(Map.newBuilder().setDisplayName("Test").build());
    Layer layer =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map.getMapId())
                .setDisplayName("layer1")
                .setType(LayerType.OCCUPANCY_GRID_LAYER)
                .build());
    stub.deleteLayer(
        DeleteLayerRequest.newBuilder()
            .setMapId(map.getMapId())
            .setLayerId(layer.getLayerId())
            .build());
    try {
      stub.getLayer(
          GetLayerRequest.newBuilder()
              .setMapId(map.getMapId())
              .setLayerId(layer.getLayerId())
              .build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }

  @Test
  public void testGetLayer_afterDeleteMap_throwsNotFound() {
    Map map = stub.createMap(Map.newBuilder().setDisplayName("Test").build());
    Layer layer =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map.getMapId())
                .setDisplayName("layer1")
                .setType(LayerType.OCCUPANCY_GRID_LAYER)
                .build());
    stub.deleteMap(DeleteMapRequest.newBuilder().setMapId(map.getMapId()).build());
    try {
      stub.getLayer(
          GetLayerRequest.newBuilder()
              .setMapId(map.getMapId())
              .setLayerId(layer.getLayerId())
              .build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }

  @Test
  public void testCreatePoint_hasId() {
    Map createMapResponse = stub.createMap(Map.newBuilder().setDisplayName("TestMap").build());
    Layer createLayerResponse =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(createMapResponse.getMapId())
                .setDisplayName("TestLayer")
                .setType(POINT_LAYER)
                .build());
    Point createPointResponse =
        stub.createPoint(
            Point.newBuilder()
                .setMapId(createMapResponse.getMapId())
                .setLayerId(createLayerResponse.getLayerId())
                .setDisplayName("TestPoint")
                .setPointPose(
                    Rigid2d.newBuilder()
                        .setTranslation(Vector2d.newBuilder().setX(1.0).setY(2.0).build())
                        .setRotation(3.0)
                        .build())
                .build());
    assertNotEquals(0, createPointResponse.getPointId());
  }

  @Test
  public void testListPoints_containsElements() {
    Map map = stub.createMap(Map.newBuilder().setDisplayName("map").build());
    Layer layer1 =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map.getMapId())
                .setDisplayName("layer1")
                .setType(POINT_LAYER)
                .build());
    Layer layer2 =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map.getMapId())
                .setDisplayName("layer2")
                .setType(POINT_LAYER)
                .build());
    Point point1 =
        stub.createPoint(
            Point.newBuilder()
                .setMapId(map.getMapId())
                .setLayerId(layer1.getLayerId())
                .setDisplayName("point1")
                .build());
    Point point2 =
        stub.createPoint(
            Point.newBuilder()
                .setMapId(map.getMapId())
                .setLayerId(layer2.getLayerId())
                .setDisplayName("point2")
                .build());
    Point point3 =
        stub.createPoint(
            Point.newBuilder()
                .setMapId(map.getMapId())
                .setLayerId(layer2.getLayerId())
                .setDisplayName("point3")
                .build());

    assertThat(
        stub.listPoints(
                ListPointsRequest.newBuilder()
                    .setMapId(map.getMapId())
                    .setLayerId(layer1.getLayerId())
                    .build())
            .getPointsList(),
        containsInAnyOrder(point1));
    assertThat(
        stub.listPoints(
                ListPointsRequest.newBuilder()
                    .setMapId(map.getMapId())
                    .setLayerId(layer2.getLayerId())
                    .build())
            .getPointsList(),
        containsInAnyOrder(point2, point3));
  }

  @Test
  public void testGetPoint_afterDeletePoint_throwsNotFound() {
    Map map = stub.createMap(Map.newBuilder().setDisplayName("map").build());
    Layer layer =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map.getMapId())
                .setDisplayName("layer")
                .setType(LayerType.POINT_LAYER)
                .build());
    Point point =
        stub.createPoint(
            Point.newBuilder()
                .setMapId(map.getMapId())
                .setLayerId(layer.getLayerId())
                .setDisplayName("point")
                .build());
    stub.deletePoint(point);
    try {
      stub.getPoint(
          GetPointRequest.newBuilder()
              .setMapId(map.getMapId())
              .setLayerId(layer.getLayerId())
              .setPointId(point.getPointId())
              .build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }

  @Test
  public void testGetPoint_afterDeleteLayer_throwsNotFound() {
    Map map = stub.createMap(Map.newBuilder().setDisplayName("map").build());
    Layer layer =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map.getMapId())
                .setDisplayName("layer")
                .setType(LayerType.POINT_LAYER)
                .build());
    Point point =
        stub.createPoint(
            Point.newBuilder()
                .setMapId(map.getMapId())
                .setLayerId(layer.getLayerId())
                .setDisplayName("point")
                .build());
    stub.deleteLayer(
        DeleteLayerRequest.newBuilder()
            .setMapId(map.getMapId())
            .setLayerId(layer.getLayerId())
            .build());
    try {
      stub.getPoint(
          GetPointRequest.newBuilder()
              .setMapId(map.getMapId())
              .setLayerId(layer.getLayerId())
              .setPointId(point.getPointId())
              .build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }

  @Test
  public void testGetOccupancyGrid_afterCreation_succeeds() {
    Map map = stub.createMap(Map.newBuilder().setDisplayName("TestMap").build());
    Layer layer =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map.getMapId())
                .setDisplayName("TestLayer")
                .setType(OCCUPANCY_GRID_LAYER)
                .build());

    OccupancyGrid occupancyGrid =
        OccupancyGrid.newBuilder()
            .setMapId(map.getMapId())
            .setLayerId(layer.getLayerId())
            .setPbstreamUrl("http://www.google.com")
            .setImageUrl("http://www.google.com/image")
            .setImagePixelLength(0.01)
            .setImageToMap(
                Rigid2d.newBuilder()
                    .setTranslation(Vector2d.newBuilder().setX(0.3).setY(-0.4).build())
                    .setRotation(0.5)
                    .build())
            .build();

    OccupancyGrid received = stub.createOccupancyGrid(occupancyGrid);
    assertEquals(received, occupancyGrid);

    OccupancyGrid receivedWithGet =
        stub.getOccupancyGrid(
            GetOccupancyGridRequest.newBuilder()
                .setMapId(map.getMapId())
                .setLayerId(layer.getLayerId())
                .build());
    assertEquals(received, receivedWithGet);
  }

  @Test
  public void testGetOccupancyGrid_afterDeleteOccupancyGrid_throwsNotFound() {
    Map map = stub.createMap(Map.newBuilder().setDisplayName("map").build());
    Layer layer =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map.getMapId())
                .setDisplayName("layer")
                .setType(LayerType.OCCUPANCY_GRID_LAYER)
                .build());
    OccupancyGrid occupancyGrid =
        stub.createOccupancyGrid(
            OccupancyGrid.newBuilder()
                .setMapId(map.getMapId())
                .setLayerId(layer.getLayerId())
                .setPbstreamUrl("http://www.google.com")
                .build());
    stub.deleteOccupancyGrid(occupancyGrid);
    try {
      stub.getOccupancyGrid(
          GetOccupancyGridRequest.newBuilder()
              .setMapId(map.getMapId())
              .setLayerId(layer.getLayerId())
              .build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }

  @Test
  public void testGetOccupancyGrid_afterDeleteLayer_throwsNotFound() {
    Map map = stub.createMap(Map.newBuilder().setDisplayName("map").build());
    Layer layer =
        stub.createLayer(
            Layer.newBuilder()
                .setMapId(map.getMapId())
                .setDisplayName("layer")
                .setType(LayerType.OCCUPANCY_GRID_LAYER)
                .build());
    stub.createOccupancyGrid(
        OccupancyGrid.newBuilder()
            .setMapId(map.getMapId())
            .setLayerId(layer.getLayerId())
            .setPbstreamUrl("http://www.google.com")
            .build());

    stub.deleteLayer(
        DeleteLayerRequest.newBuilder()
            .setMapId(map.getMapId())
            .setLayerId(layer.getLayerId())
            .build());
    try {
      stub.getOccupancyGrid(
          GetOccupancyGridRequest.newBuilder()
              .setMapId(map.getMapId())
              .setLayerId(layer.getLayerId())
              .build());
      fail();
    } catch (StatusRuntimeException e) {
      assertEquals(e.getStatus().getCode(), Code.NOT_FOUND);
    }
  }
}
