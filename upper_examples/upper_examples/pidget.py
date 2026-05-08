import rclpy
from rclpy.node import Node
from zit6_interfaces.srv import GetParams
import json
import time


def main():
    rclpy.init()
    node = rclpy.create_node('get_params_client')
    client = node.create_client(GetParams, 'zit6/get_params')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service /zit6/get_params not available')
        rclpy.shutdown()
        return

    req = GetParams.Request()
    # paths is string[] — provide a list. Empty list requests full config.
    req.paths = ['chassis.pid.pos.kp']

    future = client.call_async(req)

    # simple timeout loop while spinning the node
    timeout = 5.0
    start = time.time()
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > timeout:
            node.get_logger().error('Service call timed out')
            break

    if future.done():
        res = future.result()
        if res is None:
            node.get_logger().error('Service call failed (no response)')
        else:
            if res.success:
                try:
                    cfg = json.loads(res.config_json)
                    print(json.dumps(cfg, indent=2, ensure_ascii=False))
                except Exception:
                    print(res.config_json)
            else:
                node.get_logger().error('Service returned error: %s' % res.message)

    rclpy.shutdown()


if __name__ == '__main__':
    main()