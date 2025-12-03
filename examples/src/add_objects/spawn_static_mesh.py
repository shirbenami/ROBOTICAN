#!/usr/bin/env python3

from sphera_common_interfaces.srv import SpawnActor
import rclpy
from rclpy.node import Node
import getpass
import json


class MeshSpawner(Node):

    def __init__(self):
        super().__init__("mesh_spawner")
        user = getpass.getuser()
        user = user.replace("-", "_")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("possess_on_spawn", True), # if true - will automatically possess after spawn
                (
                    "mesh_path",
                   # "/home/user1/SpheraAssets/tag15/Tag15.obj", # change to your FBX file path
                
                    "/home/rooster/workspace/src/examples/src/add_objects/tag15_fbx/tag15.fbx"
                ),
                ("name", "AprilTag_Test_1"), # set the name of the spawned mesh
                ("segmentation_id", 0),  # (allowed values: 0-200) used for thermal signeture & for synthetic data collection - 0 means disabled
                ("x", 3.0), # meters
                ("y", 7.0), # meters
                ("z", 0.0), # meters
                ("roll", 0.0), # radians
                ("pitch", 0.0), # radians
                ("yaw", 1.79), # radians
                ("scale", 5.0), 
            ],
        )

        #self.cli = self.create_client(SpawnActor, "/sphera/" + user + "/spawn")
        self.cli = self.create_client(SpawnActor, "/sphera/user1/spawn")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Spawn service not available, waiting...")

    def send_request(
        self,
        mesh_path,
        segmentation_id,
        name,
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
        scale,
        possess_on_spawn,
    ):

        req = SpawnActor.Request()
        req.type = "StaticObject"
        #req.transform.header.frame_id = name
        req.transform.header.frame_id = "world"
        req.transform.location.x = x
        req.transform.location.y = y
        req.transform.location.z = z
        req.transform.rotation.roll = roll
        req.transform.rotation.pitch = pitch
        req.transform.rotation.yaw = yaw

        options = {
            "possess_on_spawn": possess_on_spawn,
            "mesh_path": mesh_path,
            "scale": scale,
            "segmentation_id": segmentation_id, 
        }

        req.options = json.dumps(options)

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)

        response = self.future.result()

       # self.get_logger().info(
        #    "Spawning... " + ("OK" if response.success else response.message)
        #)
        self.get_logger().info(
            f"Spawning... success={response.success}, message='{response.message}'"
        )


def main(args=None):
    rclpy.init(args=args)

    mesh_spawner = MeshSpawner()
    mesh_spawner.send_request(
        mesh_spawner.get_parameter("mesh_path").get_parameter_value().string_value,
        mesh_spawner.get_parameter("segmentation_id").get_parameter_value().integer_value,
        mesh_spawner.get_parameter("name").get_parameter_value().string_value,
        mesh_spawner.get_parameter("x").get_parameter_value().double_value,
        mesh_spawner.get_parameter("y").get_parameter_value().double_value,
        mesh_spawner.get_parameter("z").get_parameter_value().double_value,
        mesh_spawner.get_parameter("roll").get_parameter_value().double_value,
        mesh_spawner.get_parameter("pitch").get_parameter_value().double_value,
        mesh_spawner.get_parameter("yaw").get_parameter_value().double_value,
        mesh_spawner.get_parameter("scale").get_parameter_value().double_value,
        mesh_spawner.get_parameter("possess_on_spawn").get_parameter_value().bool_value,
    )

    mesh_spawner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
