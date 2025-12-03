#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from controller_manager_msgs.srv import LoadController, ConfigureController, SwitchController, ListControllers

class ControllerStarter(Node):
    def __init__(self):
        super().__init__('controller_starter')
        self.declare_parameter('controllers', ['joint_state_broadcaster','arm_controller','gripper_controller'])
        self.controllers = self.get_parameter('controllers').get_parameter_value().string_array_value
        # Clients
        self.load_client = self.create_client(LoadController, '/controller_manager/load_controller')
        self.configure_client = self.create_client(ConfigureController, '/controller_manager/configure_controller')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')

        self.get_logger().info('Waiting for controller_manager services...')
        self.wait_for_services()
        self.start_sequence()

    def wait_for_services(self):
        timeout = 30.0
        start = time.time()
        while rclpy.ok():
            ok = (self.load_client.wait_for_service(timeout_sec=1.0) and
                  self.configure_client.wait_for_service(timeout_sec=1.0) and
                  self.switch_client.wait_for_service(timeout_sec=1.0) and
                  self.list_client.wait_for_service(timeout_sec=1.0))
            if ok:
                self.get_logger().info('controller_manager services available')
                return
            if time.time() - start > timeout:
                self.get_logger().warn('Timed out waiting for controller_manager services; retrying...')
                start = time.time()
            time.sleep(0.5)

    def call_load(self, name):
        req = LoadController.Request()
        req.name = name
        fut = self.load_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() is not None:
            return fut.result().ok
        return False

    def call_configure(self, name):
        req = ConfigureController.Request()
        req.name = name
        fut = self.configure_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() is not None:
            return fut.result().ok
        return False

    def call_switch(self, activate_list):
        req = SwitchController.Request()
        req.activate_controllers = activate_list
        req.deactivate_controllers = []
        req.strictness = SwitchController.Request.STRICT
        fut = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() is not None:
            return fut.result().ok
        return False

    def list_controllers(self):
        req = ListControllers.Request()
        fut = self.list_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() is not None:
            return [c.name for c in fut.result().controller]
        return []

    def start_sequence(self):
        # Retry loop for robustness
        max_attempts = 5
        for attempt in range(max_attempts):
            self.get_logger().info(f'Controller start attempt {attempt+1}/{max_attempts}')
            existing = self.list_controllers()
            self.get_logger().info(f'Existing controllers: {existing}')
            for c in self.controllers:
                if c in existing:
                    self.get_logger().info(f"Controller '{c}' already present")
                    continue
                self.get_logger().info(f"Loading controller '{c}'")
                ok = self.call_load(c)
                self.get_logger().info(f'Load {c}: {ok}')
                if not ok:
                    # try set minimal type param via node param (best-effort)
                    try:
                        # if controller_manager node exists, set type param to help loader (`joint_state_broadcaster` etc.)
                        self.get_logger().warn(f"Failed to load {c}; will retry after 1s")
                    except Exception:
                        pass
                    continue
                # configure
                self.get_logger().info(f'Configuring {c}')
                ok_conf = self.call_configure(c)
                self.get_logger().info(f'Configure {c}: {ok_conf}')
            # Activate all
            self.get_logger().info('Activating controllers')
            ok_switch = self.call_switch(self.controllers)
            self.get_logger().info(f'Switch result: {ok_switch}')
            # Check if joint_state_broadcaster active
            existing = self.list_controllers()
            if 'joint_state_broadcaster' in existing:
                self.get_logger().info('joint_state_broadcaster present after attempt')
            # If successful break
            if ok_switch:
                self.get_logger().info('Controllers activated successfully')
                break
            time.sleep(1.0)
        # done
        self.get_logger().info('Controller starter finished')
        rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    node = ControllerStarter()
