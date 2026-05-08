import rclpy, json
from rclpy.node import Node
from zit6_interfaces.srv import UpdateParams

def main():
    rclpy.init()
    node = Node('upd_client')
    client = node.create_client(UpdateParams, 'zit6/update_params')
    if client.wait_for_service(timeout_sec=5.0):
        req = UpdateParams.Request()
        req.json = json.dumps({
            "chassis": {
                "pid": {"pos": {"kp": 0.02, "ki": 0.001}},
                "profile": {"default_max_v": 0.6}
            }
        })
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        print(future.result().success, future.result().message)
    rclpy.shutdown()

if __name__ == '__main__':
    main()