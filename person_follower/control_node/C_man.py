import os
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Parámetros
        self.declare_parameter('enabled', True)
        self.declare_parameter('slam_enabled', True)
        self.declare_parameter('camera_enabled', True)
        self.enabled        = self.get_parameter('enabled').value
        self.slam_enabled   = self.get_parameter('slam_enabled').value
        self.ignore_gesture = not self.get_parameter('camera_enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Control desactivado.")
            return

        # FSM: INIT, IDLE, TRACKING, MANUAL, SHUTDOWN
        self.states = ['INIT', 'IDLE', 'TRACKING', 'MANUAL', 'SHUTDOWN']
        self.current_state = 'INIT'

        # Flags
        self.person_detected        = False
        self.user_authorized        = self.ignore_gesture
        self.tracking_service_ready = False
        self._last_tracking_enable  = None

        # Subscripciones
        self.create_subscription(Bool,   '/person_detected',       self.person_detected_callback,       10)
        self.create_subscription(Bool,   '/shutdown_confirmation', self.shutdown_confirmation_callback, 10)
        self.create_subscription(Twist,  '/tracking/velocity_cmd', self.velocity_callback,              10)
        self.create_subscription(String, '/gesture_command',       self.gesture_command_callback,      10)

        # Publicadores y cliente de tracking
        self.shutdown_publisher = self.create_publisher(Bool,  '/system_shutdown', 10)
        self.cmd_vel_publisher  = self.create_publisher(Twist, '/cmd_vel',         10)
        self.tracking_client    = self.create_client(SetBool, 'enable_tracking')
        self.wait_for_service(self.tracking_client, 'enable_tracking')

        self.get_logger().info("Nodo de Control iniciado.")
        self.start_keyboard_listener()
        self.transition_to('IDLE')

    def wait_for_service(self, client, name):
        while not client.service_is_ready():
            self.get_logger().info(f"Esperando servicio {name}...")
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f"Servicio {name} listo.")
        self.tracking_service_ready = True

    # Callback de tracking automático (modificado)
    def velocity_callback(self, msg):
        if self.current_state == 'MANUAL':
            return  # no interferir en manual
        if self.current_state == 'TRACKING':
            self.cmd_vel_publisher.publish(msg)
        else:
            self.stop_robot()

    # Callback de gestos
    def gesture_command_callback(self, msg):
        if self.ignore_gesture:
            return
        gesture = msg.data.lower().strip()
        self.get_logger().info(f"Gesto recibido: {gesture}")
        if gesture == "start_tracking":
            self.user_authorized = True
            if self.person_detected and self.current_state == 'IDLE':
                self.transition_to('TRACKING')
        elif gesture == "stop_tracking":
            self.user_authorized = False
            if self.current_state == 'TRACKING':
                self.transition_to('IDLE')

    # Teclado: toggle MANUAL/AUTO + teleop integrado
    def start_keyboard_listener(self):
        thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        thread.start()

    def keyboard_loop(self):
        try:
            tty_fd = os.open('/dev/tty', os.O_RDONLY)
        except Exception as e:
            self.get_logger().warn(f"No hay /dev/tty: modo MANUAL deshabilitado: {e}")
            return

        old_settings = termios.tcgetattr(tty_fd)
        tty.setcbreak(tty_fd)
        self.get_logger().info("Teclado: [q] cambia MANUAL/AUTO, en MANUAL w/s/a/d/x para mover")

        try:
            while rclpy.ok():
                ready, _, _ = select.select([tty_fd], [], [], 0.1)
                if not ready:
                    continue
                ch = os.read(tty_fd, 1).decode()

                if ch == '\x03':  # Ctrl-C
                    break

                # Toggle MANUAL <-> AUTO con 'q'
                if ch.lower() == 'q':
                    if self.current_state == 'MANUAL':
                        self.get_logger().info(">> MODO AUTO")
                        self.transition_to_auto()
                    else:
                        self.get_logger().info(">> MODO MANUAL")
                        self.transition_to('MANUAL')
                    continue

                # Comandos en MANUAL
                if self.current_state == 'MANUAL':
                    cmd = Twist()
                    if ch.lower() == 'w':
                        cmd.linear.x = 0.2
                    elif ch.lower() == 's':
                        cmd.linear.x = -0.2
                    elif ch.lower() == 'a':
                        cmd.angular.z = 0.5
                    elif ch.lower() == 'd':
                        cmd.angular.z = -0.5
                    elif ch.lower() == 'x':
                        pass  # todo cero
                    else:
                        continue
                    self.cmd_vel_publisher.publish(cmd)
                    self.get_logger().info(f"[MANUAL] lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}")
        finally:
            termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_settings)
            os.close(tty_fd)

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

    # FSM y transiciones
    def transition_to(self, new_state):
        if new_state not in self.states:
            self.get_logger().error(f"Estado inválido: {new_state}")
            return
        self.get_logger().info(f"Transición {self.current_state} → {new_state}")
        self.current_state = new_state

        if new_state == 'IDLE':
            self.get_logger().info(">> IDLE")
        elif new_state == 'TRACKING':
            self.start_tracking()
        elif new_state == 'MANUAL':
            # desactiva tracking al entrar en manual
            self.toggle_tracking(False)
        elif new_state == 'SHUTDOWN':
            self.notify_shutdown()

    def transition_to_auto(self):
        if self.person_detected and self.user_authorized:
            self.transition_to('TRACKING')
        else:
            self.transition_to('IDLE')

    def start_tracking(self):
        if not self.tracking_service_ready:
            self.get_logger().error("Servicio tracking no listo; volviendo a IDLE.")
            return self.transition_to('IDLE')
        self.toggle_tracking(True)
        self.get_logger().info(">> TRACKING")

    def notify_shutdown(self):
        self.shutdown_publisher.publish(Bool(data=True))
        self.get_logger().info("Notificando shutdown.")
        self.destroy_node()

    def person_detected_callback(self, msg):
        self.person_detected = msg.data
        if self.current_state == 'IDLE' and self.person_detected and self.user_authorized:
            self.transition_to('TRACKING')
        elif self.current_state == 'TRACKING' and not self.person_detected:
            self.transition_to('IDLE')

    def shutdown_confirmation_callback(self, msg):
        if msg.data:
            self.transition_to('SHUTDOWN')

    def toggle_tracking(self, enable):
        self._last_tracking_enable = enable
        req = SetBool.Request()
        req.data = enable
        fut = self.tracking_client.call_async(req)
        fut.add_done_callback(self.handle_tracking_response)

    def handle_tracking_response(self, future):
        try:
            future.result()
            if self._last_tracking_enable:
                self.get_logger().info("Tracking habilitado.")
            else:
                self.get_logger().info("Tracking deshabilitado.")
        except Exception as e:
            self.get_logger().error(f"Error en servicio tracking: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido con Ctrl-C.")
    finally:
        node.notify_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
