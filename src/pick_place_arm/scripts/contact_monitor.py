#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as RclpyDuration
from ros_gz_interfaces.msg import Contacts
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration

class ContactMonitor(Node):
    def __init__(self):
        super().__init__('contact_monitor')
        self.get_logger().info('Contact monitor starting, subscribing to /box1/contacts')
        self.sub = self.create_subscription(Contacts, '/box1/contacts', self.cb_contacts, 10)
        self.cli_switch = self.create_client(SwitchController, '/controller_manager/switch_controller')

    def cb_contacts(self, msg: Contacts):
        # Contacts.msg contains a list of Contact with fields collision1, collision2
        for c in msg.contacts:
            c1 = c.collision1 if hasattr(c, 'collision1') else ''
            c2 = c.collision2 if hasattr(c, 'collision2') else ''
            # check for gripper collisions by substring match
            if ('l7l' in c1) or ('l7l' in c2) or ('l7r' in c1) or ('l7r' in c2):
                details = f'Collision detected between box and gripper: {c1} <-> {c2}'
                self.get_logger().error(details)
                # try to deactivate controllers to stop further motion
                if self.cli_switch.wait_for_service(timeout_sec=2.0):
                    req = SwitchController.Request()
                    req.activate_controllers = []
                    req.deactivate_controllers = ['arm_controller', 'gripper_controller']
                    req.strictness = 2
                    req.activate_asap = False
                    # set timeout to 2 seconds
                    t = Duration()
                    t.sec = 2
                    t.nanosec = 0
                    req.timeout = t
                    fut = self.cli_switch.call_async(req)
                    # don't block spin fully, attach callback
                    fut.add_done_callback(lambda f: self._on_switch_done(f, details))
                else:
                    self.get_logger().error('switch_controller service not available to deactivate controllers')
                # publish or take other action here if needed
                return

    def _on_switch_done(self, fut, details):
        try:
            res = fut.result()
            if res.ok:
                self.get_logger().info(f'Controllers deactivated after collision: {details}')
            else:
                self.get_logger().error(f'Failed to deactivate controllers after collision: {details}')
        except Exception as e:
            self.get_logger().error(f'Exception calling switch_controller: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
