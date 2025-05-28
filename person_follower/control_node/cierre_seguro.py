import os
import select
import termios
import tty
import threading
import time

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
        self.enabled = self.get_parameter('enabled').value
        self.slam_enabled = self.get_parameter('slam_enabled').value
        self.ignore_gesture = not self.get_parameter('camera_enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Control desactivado.")
            return

        # FSM: INIT, IDLE, TRACKING, MANUAL, SHUTDOWN
        self.states = ['INIT', 'IDLE', 'TRACKING', 'MANUAL', 'SHUTDOWN']
        self.current_state = 'INIT'

        # Flags
        self.person_detected = False
        self.user_authorized = self.ignore_gesture
        self.tracking_service_ready = False
        self._last_tracking_enable = None
        self.shutdown_requested = False

        # Terminal settings
        self.tty_fd = None
        self._original_tty_attrs = None

        # Subscripciones
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.create_subscription(Bool, '/shutdown_confirmation', self.shutdown_confirmation_callback, 10)
        self.create_subscription(Twist, '/tracking/velocity_cmd', self.velocity_callback, 10)
        self.create_subscription(String, '/gesture_command', self.gesture_command_callback, 10)

        # Publicadores y cliente de tracking
        self.shutdown_publisher = self.create_publisher(Bool, '/system_shutdown', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tracking_client = self.create_client(SetBool, 'enable_tracking')
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

    def velocity_callback(self, msg):
        if self.current_state == 'MANUAL':
            return
        if self.current_state == 'TRACKING':
            self.cmd_vel_publisher.publish(msg)
        else:
            self.stop_robot()

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

    def start_keyboard_listener(self):
        thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        thread.start()

    def keyboard_loop(self):
        try:
            # Abrir descriptor para lectura y escritura
            self.tty_fd = os.open('/dev/tty', os.O_RDWR)
            # Guardar configuración original
            self._original_tty_attrs = termios.tcgetattr(self.tty_fd)
            # Poner en modo cbreak (sin buffering y sin eco)
            tty.setcbreak(self.tty_fd)
        except Exception as e:
            self.get_logger().warn(f"No hay /dev/tty: modo MANUAL/desconexión deshabilitados: {e}")
            return

        self.get_logger().info(
            "Teclado: [q] MANUAL/AUTO, [p] PARAR SISTEMA, en MANUAL w/s/a/d/x para mover"
        )

        try:
            while rclpy.ok():
                ready, _, _ = select.select([self.tty_fd], [], [], 0.1)
                if not ready:
                    continue
                try:
                    ch = os.read(self.tty_fd, 1).decode()
                except OSError as e:
                    self.get_logger().warn(f"Lectura de teclado falló: {e}")
                    break

                if ch == '\x03':  # Ctrl-C
                    break

                if ch.lower() == 'p':
                    self.get_logger().info(">> SHUTDOWN solicitado (tecla P).")
                    self.shutdown_requested = True
                    break  # salir del bucle

                if ch.lower() == 'q':
                    if self.current_state == 'MANUAL':
                        self.get_logger().info(">> MODO AUTO")
                        self.transition_to_auto()
                    else:
                        self.get_logger().info(">> MODO MANUAL")
                        self.transition_to('MANUAL')
                    continue

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
                        pass
                    else:
                        continue
                    self.cmd_vel_publisher.publish(cmd)
                    self.get_logger().info(f"[MANUAL] lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}")
        finally:
            # Restaurar terminal solo en main(), aquí solo invertir shutdown
            if self.shutdown_requested:
                self.notify_shutdown()

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

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
        self.get_logger().info("Notificando shutdown a los nodos secundarios.")
        time.sleep(1.0)
        self.get_logger().info("Cerrando ControlNode.")
        self.destroy_node()

    def person_detected_callback(self, msg):
        self.person_detected = msg.data
        if self.current_state == 'IDLE' and self.person_detected and self.user_authorized:
            self.transition_to('TRACKING')
        elif self.current_state == 'TRACKING' and not self.person_detected:
            self.transition_to('IDLE')

    def shutdown_confirmation_callback(self, msg):
        if msg.data:
            self.get_logger().info("Confirmación de shutdown recibida.")

    def toggle_tracking(self, enable):
        self._last_tracking_enable = enable
        req = SetBool.Request()
        req.data = enable
        fut = self.tracking_client.call_async(req)
        fut.add_done_callback(self.handle_tracking_response)

    def handle_tracking_response(self, future):
        try:
            future.result()
            msg = "Tracking habilitado." if self._last_tracking_enable else "Tracking deshabilitado."
            self.get_logger().info(msg)
        except Exception as e:
            self.get_logger().error(f"Error en servicio tracking: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ControlNode detenido con Ctrl-C.")
    finally:
        if hasattr(node, 'tty_fd') and node.tty_fd and node._original_tty_attrs:
            try:
                termios.tcsetattr(node.tty_fd, termios.TCSADRAIN, node._original_tty_attrs)
                os.close(node.tty_fd)
                node.get_logger().info("Terminal restaurado correctamente desde main().")
            except Exception as e:
                print(f"[ERROR] Fallo al restaurar terminal desde main: {e}")

        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"[ERROR] shutdown falló: {e}")

if __name__ == '__main__':
    main()
