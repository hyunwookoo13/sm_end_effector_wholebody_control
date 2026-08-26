from __future__ import annotations

import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from sm_semantic_map.semantic_db import SemanticMapDB, normalize_name
from sm_semantic_map_interfaces.srv import FindObject


class SemanticMapServer(Node):
    def __init__(self) -> None:
        super().__init__("semantic_map_server")
        default_data_dir = Path.home() / ".ros" / "sm_semantic_map"
        default_seed = (
            Path(get_package_share_directory("sm_semantic_map"))
            / "config"
            / "warehouse_objects.yaml"
        )
        self.declare_parameter("db_path", str(default_data_dir / "semantic_map.sqlite3"))
        self.declare_parameter("seed_path", str(default_seed))
        self.declare_parameter("seed_database_when_empty", True)
        self.declare_parameter("find_service", "/semantic_map/find_object")

        db_path = Path(str(self.get_parameter("db_path").value)).expanduser()
        seed_path = Path(str(self.get_parameter("seed_path").value)).expanduser()
        db_path.parent.mkdir(parents=True, exist_ok=True)
        self.database = SemanticMapDB(db_path)
        if (
            bool(self.get_parameter("seed_database_when_empty").value)
            and self.database.count_objects() == 0
        ):
            loaded = self.database.load_seed(seed_path)
            self.get_logger().info(f"Seeded semantic database with {loaded} objects")

        service_name = str(self.get_parameter("find_service").value)
        self.service = self.create_service(FindObject, service_name, self._find_object)
        self.get_logger().info(
            f"Semantic map ready: db={db_path}, objects={self.database.count_objects()}, "
            f"service={service_name}"
        )

    def _find_object(
        self,
        request: FindObject.Request,
        response: FindObject.Response,
    ) -> FindObject.Response:
        query = normalize_name(request.query)
        required_capability = normalize_name(request.required_capability)
        if not query:
            response.message = "query is empty"
            return response

        record = self.database.find(query)
        if record is None:
            response.message = f"object not found: {query}"
            return response
        capabilities = [str(value) for value in record["capabilities"]]
        if required_capability and required_capability not in capabilities:
            response.message = (
                f"object {record['object_id']} does not support {required_capability}"
            )
            return response

        response.found = True
        response.message = "found"
        response.object_id = str(record["object_id"])
        response.canonical_name = str(record["canonical_name"])
        response.semantic_class = str(record["semantic_class"])
        response.capabilities = capabilities
        response.status = str(record["status"])
        response.zone = str(record["zone"])
        response.source_sensor = str(record["source_sensor"])
        response.approach_clearance_m = float(record["approach_clearance_m"])

        stamp = self.get_clock().now().to_msg()
        frame_id = str(record["frame_id"])
        response.object_pose.header.stamp = stamp
        response.object_pose.header.frame_id = frame_id
        response.object_pose.pose.position.x = float(record["object_x"])
        response.object_pose.pose.position.y = float(record["object_y"])
        response.object_pose.pose.position.z = float(record["object_z"])
        response.object_pose.pose.orientation.w = 1.0

        yaw = float(record["approach_yaw"])
        response.approach_pose.header.stamp = stamp
        response.approach_pose.header.frame_id = frame_id
        response.approach_pose.pose.position.x = float(record["approach_x"])
        response.approach_pose.pose.position.y = float(record["approach_y"])
        response.approach_pose.pose.orientation.z = math.sin(yaw * 0.5)
        response.approach_pose.pose.orientation.w = math.cos(yaw * 0.5)
        return response

    def destroy_node(self) -> bool:
        self.database.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
