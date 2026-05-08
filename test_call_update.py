#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from zit6_interfaces.srv import UpdateParams, GetParams

def main():
    rclpy.init()
    node = Node('test_update_client')

    update_client = node.create_client(UpdateParams, '/zit6/update_params')
    get_client = node.create_client(GetParams, '/zit6/get_params')

    if not update_client.wait_for_service(timeout_sec=5.0):
        print('update_params service not available', file=sys.stderr)
    else:
        # First try using paths/values (avoid JSON field serialization issues)
        req = UpdateParams.Request()
        req.paths = ['chassis.pid.pos.kp']
        req.values = ['0.5']
        future = update_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.done():
            res = future.result()
            print('Update response:', res.success, repr(res.message))
        else:
            print('Update call timed out', file=sys.stderr)

        # Also try JSON form as a comparison
        req2 = UpdateParams.Request()
        req2.json = '{"chassis":{"pid":{"pos":{"kp":0.5}}}}'
        fut2 = update_client.call_async(req2)
        rclpy.spin_until_future_complete(node, fut2, timeout_sec=5.0)
        if fut2.done():
            r2 = fut2.result()
            print('Update (json) response:', r2.success, repr(r2.message))
        else:
            print('Update (json) call timed out', file=sys.stderr)

    if not get_client.wait_for_service(timeout_sec=5.0):
        print('get_params service not available', file=sys.stderr)
    else:
        req2 = GetParams.Request()
        future2 = get_client.call_async(req2)
        rclpy.spin_until_future_complete(node, future2, timeout_sec=5.0)
        if future2.done():
            res2 = future2.result()
            print('Get response success:', res2.success)
            print('config_json:', res2.config_json)
        else:
            print('Get call timed out', file=sys.stderr)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
