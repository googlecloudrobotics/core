// Copyright 2019 The Google Cloud Robotics Authors
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

import com.cloudrobotics.common.v1alpha1.Rigid2d;
import com.cloudrobotics.common.v1alpha1.Vector2d;
import com.cloudrobotics.map.v1alpha1.DeleteLayerRequest;
import com.cloudrobotics.map.v1alpha1.DeleteMapRequest;
import com.cloudrobotics.map.v1alpha1.GetLayerRequest;
import com.cloudrobotics.map.v1alpha1.GetMapRequest;
import com.cloudrobotics.map.v1alpha1.GetOccupancyGridRequest;
import com.cloudrobotics.map.v1alpha1.GetPointRequest;
import com.cloudrobotics.map.v1alpha1.Layer;
import com.cloudrobotics.map.v1alpha1.Layer.LayerType;
import com.cloudrobotics.map.v1alpha1.ListLayersRequest;
import com.cloudrobotics.map.v1alpha1.ListLayersResponse;
import com.cloudrobotics.map.v1alpha1.ListMapsResponse;
import com.cloudrobotics.map.v1alpha1.ListPointsRequest;
import com.cloudrobotics.map.v1alpha1.ListPointsResponse;
import com.cloudrobotics.map.v1alpha1.Map;
import com.cloudrobotics.map.v1alpha1.MapServiceGrpc;
import com.cloudrobotics.map.v1alpha1.OccupancyGrid;
import com.cloudrobotics.map.v1alpha1.Point;
import com.google.cloud.Timestamp;
import com.google.cloud.datastore.Datastore;
import com.google.cloud.datastore.Entity;
import com.google.cloud.datastore.Key;
import com.google.cloud.datastore.PathElement;
import com.google.cloud.datastore.Query;
import com.google.cloud.datastore.StructuredQuery.OrderBy;
import com.google.cloud.datastore.StructuredQuery.PropertyFilter;
import com.google.common.collect.ImmutableList;
import com.google.common.flogger.FluentLogger;
import com.google.protobuf.Empty;
import io.grpc.Status;
import io.grpc.StatusRuntimeException;
import io.grpc.stub.StreamObserver;
import java.util.Iterator;
import java.util.Optional;

public class MapService extends MapServiceGrpc.MapServiceImplBase {

  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  private static final String DATASTORE_MAP_KEY_KIND = "Map";
  private static final String DATASTORE_LAYER_KEY_KIND = "Layer";
  private static final String DATASTORE_POINT_KEY_KIND = "Point";
  private static final String DATASTORE_OCCUPANCY_GRID_KEY_KIND = "OccupancyGrid";

  private final Datastore datastore;

  public MapService(Datastore datastore) {
    super();
    this.datastore = datastore;
  }

  private static Map toMapProto(Entity map) {
    return Map.newBuilder()
        .setMapId(map.getKey().getId())
        .setCreateTime(map.getTimestamp("create_time").toProto())
        .setDisplayName(map.getString("display_name"))
        .build();
  }

  private static Layer toLayerProto(Entity layer) {
    return Layer.newBuilder()
        .setMapId(layer.getKey("map_id").getId())
        .setLayerId(layer.getKey().getId())
        .setCreateTime(layer.getTimestamp("create_time").toProto())
        .setDisplayName(layer.getString("display_name"))
        .setType(LayerType.valueOf(layer.getString("type")))
        .build();
  }

  private static Point toPointProto(Entity point) {
    return Point.newBuilder()
        .setMapId(point.getKey("map_id").getId())
        .setLayerId(point.getKey("layer_id").getId())
        .setPointId(point.getKey().getId())
        .setCreateTime(point.getTimestamp("create_time").toProto())
        .setDisplayName(point.getString("display_name"))
        .setPointPose(
            Rigid2d.newBuilder()
                .setTranslation(
                    Vector2d.newBuilder()
                        .setX(point.getDouble("position.x"))
                        .setY(point.getDouble("position.y"))
                        .build())
                .setRotation(point.getDouble("rotation"))
                .build())
        .build();
  }

  private static OccupancyGrid toOccupancyGridProto(Entity occupancyGrid) {
    return OccupancyGrid.newBuilder()
        .setMapId(occupancyGrid.getKey("map_id").getId())
        .setLayerId(occupancyGrid.getKey("layer_id").getId())
        .setPbstreamUrl(occupancyGrid.getString("pbstream_url"))
        .setImageUrl(occupancyGrid.getString("image_url"))
        .setImagePixelLength(occupancyGrid.getDouble("image_pixel_length"))
        .setImageToMap(
            Rigid2d.newBuilder()
                .setTranslation(
                    Vector2d.newBuilder()
                        .setX(occupancyGrid.getDouble("image_to_map.translation.x"))
                        .setY(occupancyGrid.getDouble("image_to_map.translation.y"))
                        .build())
                .setRotation(occupancyGrid.getDouble("image_to_map.rotation"))
                .build())
        .build();
  }

  @Override
  public void createMap(Map request, StreamObserver<Map> responseObserver) {
    Key key =
        datastore.allocateId(datastore.newKeyFactory().setKind(DATASTORE_MAP_KEY_KIND).newKey());
    Entity map =
        Entity.newBuilder(key)
            .set("create_time", Timestamp.now())
            .set("display_name", request.getDisplayName())
            .build();

    map = datastore.add(map);

    responseObserver.onNext(toMapProto(map));
    responseObserver.onCompleted();
  }

  @Override
  public void getMap(GetMapRequest request, StreamObserver<Map> responseObserver) {
    Key key = datastore.newKeyFactory().setKind(DATASTORE_MAP_KEY_KIND).newKey(request.getMapId());
    Entity map = datastore.get(key);
    if (map == null) {
      responseObserver.onError(
          new StatusRuntimeException(
              Status.NOT_FOUND.withDescription("Map " + request.getMapId() + " not found.")));
      return;
    }

    responseObserver.onNext(toMapProto(map));
    responseObserver.onCompleted();
  }

  @Override
  public void listMaps(Empty request, StreamObserver<ListMapsResponse> responseObserver) {
    Query<Entity> query =
        Query.newEntityQueryBuilder()
            .setKind(DATASTORE_MAP_KEY_KIND)
            .setOrderBy(OrderBy.asc("create_time"))
            .build();
    Iterator<Entity> maps = datastore.run(query);
    ListMapsResponse.Builder responseBuilder = ListMapsResponse.newBuilder();
    while (maps.hasNext()) {
      responseBuilder.addMaps(toMapProto(maps.next()));
    }
    responseObserver.onNext(responseBuilder.build());
    responseObserver.onCompleted();
  }

  @Override
  public void deleteMap(DeleteMapRequest request, StreamObserver<Empty> responseObserver) {
    ImmutableList<Layer> layers = listLayers(request.getMapId());
    layers.forEach(l -> deleteLayer(l.getMapId(), l.getLayerId()));

    datastore.delete(
        datastore.newKeyFactory().setKind(DATASTORE_MAP_KEY_KIND).newKey(request.getMapId()));
    responseObserver.onNext(Empty.getDefaultInstance());
    responseObserver.onCompleted();
  }

  @Override
  public void createLayer(Layer request, StreamObserver<Layer> responseObserver) {
    if (request.getMapId() == 0) {
      responseObserver.onError(
          new StatusRuntimeException(Status.INVALID_ARGUMENT.withDescription("Must set map_id.")));
      return;
    }
    Key key =
        datastore.allocateId(
            datastore
                .newKeyFactory()
                .setKind(DATASTORE_LAYER_KEY_KIND)
                .addAncestor(PathElement.of(DATASTORE_MAP_KEY_KIND, request.getMapId()))
                .newKey());
    Key mapKey =
        datastore.newKeyFactory().setKind(DATASTORE_MAP_KEY_KIND).newKey(request.getMapId());
    Entity layer =
        Entity.newBuilder(key)
            .set("map_id", mapKey)
            .set("create_time", Timestamp.now())
            .set("display_name", request.getDisplayName())
            .set("type", request.getType().toString())
            .build();

    layer = datastore.add(layer);

    responseObserver.onNext(toLayerProto(layer));
    responseObserver.onCompleted();
  }

  private Layer getLayer(long mapId, long layerId) {
    logger.atInfo().log("Getting Layer(MapID: %d, LayerID: %d)", mapId, layerId);
    Key key =
        datastore
            .newKeyFactory()
            .setKind(DATASTORE_LAYER_KEY_KIND)
            .addAncestor(PathElement.of(DATASTORE_MAP_KEY_KIND, mapId))
            .newKey(layerId);
    Entity layer = datastore.get(key);
    if (layer == null) {
      return null;
    }
    return toLayerProto(layer);
  }

  @Override
  public void getLayer(GetLayerRequest request, StreamObserver<Layer> responseObserver) {
    Layer layer = getLayer(request.getMapId(), request.getLayerId());
    if (layer == null || layer.getMapId() != request.getMapId()) {
      responseObserver.onError(
          new StatusRuntimeException(
              Status.NOT_FOUND.withDescription("Layer " + request.getLayerId() + " not found.")));
      return;
    }
    responseObserver.onNext(layer);
    responseObserver.onCompleted();
  }

  private ImmutableList<Layer> listLayers(long mapId) {
    logger.atInfo().log("Listing Layers (MapID: %d)", mapId);
    Key mapKey = datastore.newKeyFactory().setKind(DATASTORE_MAP_KEY_KIND).newKey(mapId);
    Query<Entity> query =
        Query.newEntityQueryBuilder()
            .setKind(DATASTORE_LAYER_KEY_KIND)
            .setFilter(PropertyFilter.hasAncestor(mapKey))
            .build();
    ImmutableList.Builder listBuilder = ImmutableList.builder();
    datastore.run(query).forEachRemaining(l -> listBuilder.add(toLayerProto(l)));
    return listBuilder.build();
  }

  @Override
  public void listLayers(
      ListLayersRequest request, StreamObserver<ListLayersResponse> responseObserver) {
    ImmutableList<Layer> layers = listLayers(request.getMapId());
    ListLayersResponse.Builder responseBuilder = ListLayersResponse.newBuilder();
    responseBuilder.addAllLayers(layers);
    responseObserver.onNext(responseBuilder.build());
    responseObserver.onCompleted();
  }

  private void deleteLayer(long mapId, long layerId) throws StatusRuntimeException {
    Layer layer = getLayer(mapId, layerId);
    if (layer == null) {
      throw new StatusRuntimeException(
          Status.NOT_FOUND.withDescription(
              String.format("Layer (MapID: %d, LayerID: %d) not found.", mapId, layerId)));
    }
    switch (layer.getType()) {
      case POINT_LAYER:
        ImmutableList<Point> points = listPoints(layer.getMapId(), layer.getLayerId());
        points.forEach(l -> deletePoint(l.getMapId(), l.getLayerId(), l.getPointId()));
        break;
      case OCCUPANCY_GRID_LAYER:
        deleteOccupancyGrid(layer.getMapId(), layer.getLayerId());
        break;
      case ZONE_LAYER:
        // TODO(cschuet): implement me.
        break;
      default:
        throw new StatusRuntimeException(
            Status.INTERNAL.withDescription("not implemented: " + layer.getType()));
    }
    datastore.delete(
        datastore
            .newKeyFactory()
            .setKind(DATASTORE_LAYER_KEY_KIND)
            .addAncestor(PathElement.of(DATASTORE_MAP_KEY_KIND, mapId))
            .newKey(layerId));
  }

  @Override
  public void deleteLayer(DeleteLayerRequest request, StreamObserver<Empty> responseObserver) {
    try {
      deleteLayer(request.getMapId(), request.getLayerId());
    } catch (StatusRuntimeException e) {
      responseObserver.onError(e);
      return;
    }
    responseObserver.onNext(Empty.getDefaultInstance());
    responseObserver.onCompleted();
  }

  @Override
  public void createPoint(Point request, StreamObserver<Point> responseObserver) {
    Key mapKey =
        datastore.newKeyFactory().setKind(DATASTORE_MAP_KEY_KIND).newKey(request.getMapId());
    Key layerKey =
        datastore.newKeyFactory().setKind(DATASTORE_LAYER_KEY_KIND).newKey(request.getLayerId());
    Key key =
        datastore.allocateId(
            datastore
                .newKeyFactory()
                .setKind(DATASTORE_POINT_KEY_KIND)
                .addAncestors(
                    PathElement.of(DATASTORE_MAP_KEY_KIND, request.getMapId()),
                    PathElement.of(DATASTORE_LAYER_KEY_KIND, request.getLayerId()))
                .newKey());

    // Retrieve the layer to check layer type.
    Layer layer = getLayer(request.getMapId(), request.getLayerId());
    if (layer == null) {
      responseObserver.onError(
          new StatusRuntimeException(
              Status.NOT_FOUND.withDescription(
                  String.format(
                      "Layer (MapID: %d, LayerID: %d) not found.",
                      request.getMapId(), request.getLayerId()))));
    }
    if (layer.getType() != LayerType.POINT_LAYER) {
      responseObserver.onError(
          new StatusRuntimeException(
              Status.INVALID_ARGUMENT.withDescription(
                  "Layer " + request.getLayerId() + " is not a POINT_LAYER.")));
      return;
    }

    Entity point =
        Entity.newBuilder(key)
            .set("map_id", mapKey)
            .set("layer_id", layerKey)
            .set("create_time", Timestamp.now())
            .set("display_name", request.getDisplayName())
            .set("position.x", request.getPointPose().getTranslation().getX())
            .set("position.y", request.getPointPose().getTranslation().getY())
            .set("rotation", request.getPointPose().getRotation())
            .build();

    point = datastore.add(point);

    responseObserver.onNext(toPointProto(point));
    responseObserver.onCompleted();
  }

  public Point getPoint(long mapId, long layerId, long pointId) {
    Key key =
        datastore
            .newKeyFactory()
            .setKind(DATASTORE_POINT_KEY_KIND)
            .addAncestors(
                PathElement.of(DATASTORE_MAP_KEY_KIND, mapId),
                PathElement.of(DATASTORE_LAYER_KEY_KIND, layerId))
            .newKey(pointId);
    Entity point = datastore.get(key);
    if (point == null) {
      return null;
    }
    return toPointProto(point);
  }

  @Override
  public void getPoint(GetPointRequest request, StreamObserver<Point> responseObserver) {
    Point point = getPoint(request.getMapId(), request.getLayerId(), request.getPointId());
    if (point == null
        || point.getMapId() != request.getMapId()
        || point.getLayerId() != request.getLayerId()) {
      responseObserver.onError(
          new StatusRuntimeException(
              Status.NOT_FOUND.withDescription("Point " + request.getPointId() + " not found.")));
      return;
    }
    responseObserver.onNext(point);
    responseObserver.onCompleted();
  }

  private ImmutableList<Point> listPoints(long mapId, long layerId) {
    Query<Entity> query =
        Query.newEntityQueryBuilder()
            .setKind(DATASTORE_POINT_KEY_KIND)
            .setFilter(
                PropertyFilter.hasAncestor(
                    datastore
                        .newKeyFactory()
                        .setKind(DATASTORE_LAYER_KEY_KIND)
                        .addAncestor(PathElement.of(DATASTORE_MAP_KEY_KIND, mapId))
                        .newKey(layerId)))
            .setOrderBy(OrderBy.asc("create_time"))
            .build();
    ImmutableList.Builder listBuilder = ImmutableList.builder();
    datastore.run(query).forEachRemaining(p -> listBuilder.add(toPointProto(p)));
    return listBuilder.build();
  }

  @Override
  public void listPoints(
      ListPointsRequest request, StreamObserver<ListPointsResponse> responseObserver) {
    ImmutableList<Point> points = listPoints(request.getMapId(), request.getLayerId());
    ListPointsResponse.Builder responseBuilder = ListPointsResponse.newBuilder();
    responseBuilder.addAllPoints(points);
    responseObserver.onNext(responseBuilder.build());
    responseObserver.onCompleted();
  }

  private void deletePoint(long mapId, long layerId, long pointId) {
    datastore.delete(
        datastore
            .newKeyFactory()
            .addAncestors(
                PathElement.of(DATASTORE_MAP_KEY_KIND, mapId),
                PathElement.of(DATASTORE_LAYER_KEY_KIND, layerId))
            .setKind(DATASTORE_POINT_KEY_KIND)
            .newKey(pointId));
  }

  @Override
  public void deletePoint(Point request, StreamObserver<Empty> responseObserver) {
    deletePoint(request.getMapId(), request.getLayerId(), request.getPointId());
    responseObserver.onNext(Empty.getDefaultInstance());
    responseObserver.onCompleted();
  }

  @Override
  public void createOccupancyGrid(
      OccupancyGrid request, StreamObserver<OccupancyGrid> responseObserver) {

    // Retrieve the layer to check layer type.
    Layer layer = getLayer(request.getMapId(), request.getLayerId());
    if (layer == null) {
      responseObserver.onError(
          new StatusRuntimeException(
              Status.NOT_FOUND.withDescription(
                  String.format(
                      "Layer (MapID: %d, LayerID: %d) not found.",
                      request.getMapId(), request.getLayerId()))));
    }
    if (layer.getType() != LayerType.OCCUPANCY_GRID_LAYER) {
      responseObserver.onError(
          new StatusRuntimeException(
              Status.INVALID_ARGUMENT.withDescription(
                  "Layer " + request.getLayerId() + " is not a OCCUPANCY_GRID_LAYER.")));
      return;
    }

    Key key =
        datastore.allocateId(
            datastore
                .newKeyFactory()
                .addAncestors(
                    PathElement.of(DATASTORE_MAP_KEY_KIND, request.getMapId()),
                    PathElement.of(DATASTORE_LAYER_KEY_KIND, request.getLayerId()))
                .setKind(DATASTORE_OCCUPANCY_GRID_KEY_KIND)
                .newKey());
    Key mapKey =
        datastore.newKeyFactory().setKind(DATASTORE_MAP_KEY_KIND).newKey(request.getMapId());
    Key layerKey =
        datastore.newKeyFactory().setKind(DATASTORE_LAYER_KEY_KIND).newKey(request.getLayerId());
    Entity occupancyGrid =
        Entity.newBuilder(key)
            .set("map_id", mapKey)
            .set("layer_id", layerKey)
            .set("pbstream_url", request.getPbstreamUrl())
            .set("image_url", request.getImageUrl())
            .set("image_pixel_length", request.getImagePixelLength())
            .set("image_to_map.translation.x", request.getImageToMap().getTranslation().getX())
            .set("image_to_map.translation.y", request.getImageToMap().getTranslation().getY())
            .set("image_to_map.rotation", request.getImageToMap().getRotation())
            .build();

    occupancyGrid = datastore.add(occupancyGrid);

    responseObserver.onNext(toOccupancyGridProto(occupancyGrid));
    responseObserver.onCompleted();
  }

  private Optional<OccupancyGrid> getOccupancyGrid(long mapId, long layerId)
      throws StatusRuntimeException {
    Key layerKey =
        datastore
            .newKeyFactory()
            .setKind(DATASTORE_LAYER_KEY_KIND)
            .addAncestor(PathElement.of(DATASTORE_MAP_KEY_KIND, mapId))
            .newKey(layerId);
    Query<Entity> query =
        Query.newEntityQueryBuilder()
            .setKind(DATASTORE_OCCUPANCY_GRID_KEY_KIND)
            .setFilter(PropertyFilter.hasAncestor(layerKey))
            .build();
    ImmutableList.Builder listBuilder = ImmutableList.builder();
    datastore.run(query).forEachRemaining(o -> listBuilder.add(toOccupancyGridProto(o)));
    ImmutableList<OccupancyGrid> occupancyGrids = listBuilder.build();
    switch (occupancyGrids.size()) {
      case 0:
        return Optional.empty();
      case 1:
        return Optional.of(occupancyGrids.get(0));
      default:
        throw new StatusRuntimeException(
            Status.INTERNAL.withDescription(
                "Layer " + layerId + " has more than 1 occupancy grid."));
    }
  }

  @Override
  public void getOccupancyGrid(
      GetOccupancyGridRequest request, StreamObserver<OccupancyGrid> responseObserver) {
    Optional<OccupancyGrid> occupancyGrid;
    try {
      occupancyGrid = getOccupancyGrid(request.getMapId(), request.getLayerId());
    } catch (StatusRuntimeException e) {
      responseObserver.onError(e);
      return;
    }
    if (!occupancyGrid.isPresent()) {
      responseObserver.onError(
          new StatusRuntimeException(
              Status.NOT_FOUND.withDescription(
                  "Occupancy grid for layer " + request.getLayerId() + " not found.")));
      return;
    }
    responseObserver.onNext(occupancyGrid.get());
    responseObserver.onCompleted();
  }

  private void deleteOccupancyGrid(long mapId, long layerId) {
    logger.atInfo().log("Deleting occupancy grid layer %s %s", mapId, layerId);
    Key layerKey =
        datastore
            .newKeyFactory()
            .setKind(DATASTORE_LAYER_KEY_KIND)
            .addAncestor(PathElement.of(DATASTORE_MAP_KEY_KIND, mapId))
            .newKey(layerId);
    Query<Entity> query =
        Query.newEntityQueryBuilder()
            .setKind(DATASTORE_OCCUPANCY_GRID_KEY_KIND)
            .setFilter(PropertyFilter.hasAncestor(layerKey))
            .build();
    datastore.run(query).forEachRemaining(o -> datastore.delete(o.getKey()));
  }

  @Override
  public void deleteOccupancyGrid(OccupancyGrid request, StreamObserver<Empty> responseObserver) {
    deleteOccupancyGrid(request.getMapId(), request.getLayerId());
    responseObserver.onNext(Empty.getDefaultInstance());
    responseObserver.onCompleted();
  }
}
