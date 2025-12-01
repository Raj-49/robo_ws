#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, LoadController, SwitchController, ConfigureController
from builtin_interfaces.msg import Duration

class ControllerSpawner(Node):

    # Phase 1 : Create Ros2 Node --------------------------------------------------------------------------------------------------------------------

    def __init__(self):
        super().__init__('wait_and_spawn')
        self.get_logger().info('Controller spawner starting')

    # Phase 2 : wait for 4 services to be avalilable ------------------------------------------------------------------------------------------------

    def wait_for_services(self, timeout=30.0):
        # wait for controller_manager services
        self.get_logger().info('Waiting for controller_manager services...')
        cli_list = self.create_client(ListControllers, '/controller_manager/list_controllers')
        cli_load = self.create_client(LoadController, '/controller_manager/load_controller')
        cli_switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        cli_configure = self.create_client(ConfigureController, '/controller_manager/configure_controller')
        start = time.time()
        while rclpy.ok():
            if (cli_list.wait_for_service(timeout_sec=1.0) and cli_load.wait_for_service(timeout_sec=1.0)
                    and cli_switch.wait_for_service(timeout_sec=1.0) and cli_configure.wait_for_service(timeout_sec=1.0)):
                self.get_logger().info('/controller_manager services are available')
                return cli_list, cli_load, cli_switch, cli_configure
            if time.time() - start > timeout:
                self.get_logger().warn('Timed out waiting for controller_manager services')
                return None, None, None, None
        return None, None, None, None

    # Phase 3 : Sends a request → returns all controllers currently known ---------------------------------------------------------------------------

    def list_controllers(self, cli_list):
        req = ListControllers.Request()
        fut = cli_list.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() is not None:
            controllers = fut.result().controller
            self.get_logger().info(f'Existing controllers: {[c.name+":"+c.state for c in controllers]}')
            return controllers
        self.get_logger().warn('Failed to call list_controllers')
        return []

    # Phase 4 : Tells controller_manager:Load this controller plugin into memory --------------------------------------------------------------------

    def load_controller(self, cli_load, name):
        req = LoadController.Request()
        req.name = name
        fut = cli_load.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() is not None:
            ok = fut.result().ok
            self.get_logger().info(f'load_controller {name} returned ok={ok}')
            return ok
        self.get_logger().error(f'load_controller {name} failed or timed out')
        return False

    # Phase 5 : Moves controller from unconfigured → inactive ---------------------------------------------------------------------------------------

    def configure_controller(self, cli_configure, name, retries=3):
        req = ConfigureController.Request()
        req.name = name
        for i in range(retries):
            fut = cli_configure.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.done() and fut.result() is not None:
                ok = fut.result().ok
                self.get_logger().info(f'configure_controller {name} returned ok={ok}')
                if ok:
                    return True
            self.get_logger().warn(f'configure_controller {name} attempt {i+1} failed; retrying')
            time.sleep(0.5)
        self.get_logger().error(f'configure_controller {name} failed after {retries} attempts')
        return False

    #  Phase 6 : It sends: controllers to activate ,controllers to deactivate -----------------------------------------------------------------------

    def switch_controllers(self, cli_switch, activate_controllers, deactivate_controllers, strictness=2, timeout_sec=5.0):
        req = SwitchController.Request()
        # use the new activate/deactivate fields (spawner may still accept start/stop but prefer activate)
        req.activate_controllers = activate_controllers
        req.deactivate_controllers = deactivate_controllers
        req.strictness = strictness
        req.activate_asap = True
        # timeout is of type builtin_interfaces/Duration
        d = Duration()
        d.sec = int(timeout_sec)
        d.nanosec = int((timeout_sec - int(timeout_sec)) * 1e9)
        req.timeout = d
        fut = cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.done() and fut.result() is not None:
            ok = fut.result().ok
            self.get_logger().info(f'switch_controller activate={activate_controllers} deactivate={deactivate_controllers} ok={ok}')
            return ok
        self.get_logger().error('switch_controller call failed or timed out')
        return False

# Main Function -------------------------------------------------------------------------------------------------------------------------------------

def main():

    # Step 1 — Activate joint_state_broadcaster -----------------------------------------------------------------------------------------------------

    rclpy.init()
    node = ControllerSpawner()

    # Step 2 — Wait for /controller_manager to be ready ---------------------------------------------------------------------------------------------

    cli_list, cli_load, cli_switch, cli_configure = node.wait_for_services(timeout=40.0)
    if cli_list is None:
        node.get_logger().error('Controller manager services not available; exiting')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Step 3 — List/log existing controllers --------------------------------------------------------------------------------------------------------

    existing = node.list_controllers(cli_list)
    controllers = ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']

    # Step 4 — Load missing controllers -------------------------------------------------------------------------------------------------------------

    for ctrl in controllers:
        found = any(c.name == ctrl for c in existing)
        if found:
            node.get_logger().info(f'Controller {ctrl} already present')
        else:
            node.get_logger().info(f'Loading controller {ctrl}')
            ok = node.load_controller(cli_load, ctrl)
            if not ok:
                node.get_logger().warn(f'Controller {ctrl} failed to load; check ros2_control.yaml and controller type availability')
            time.sleep(0.5)

    #  Step 5 — Configure all controllers (required before activation) ------------------------------------------------------------------------------

    for ctrl in controllers:
        node.get_logger().info(f'Configuring controller {ctrl}')
        node.configure_controller(cli_configure, ctrl)
        time.sleep(0.2)

    # Step 6 — Activate them in the correct order ---------------------------------------------------------------------------------------------------

    node.switch_controllers(cli_switch, ['joint_state_broadcaster'], [], strictness=2, timeout_sec=5.0)
    time.sleep(0.5)
    node.switch_controllers(cli_switch, ['arm_controller','gripper_controller'], [], strictness=2, timeout_sec=5.0)

    # Step 7 — Final list + shutdown -----------------------------------------------------------------------------------------------------------------\

    node.list_controllers(cli_list)
    node.get_logger().info('Controller spawn sequence finished')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
