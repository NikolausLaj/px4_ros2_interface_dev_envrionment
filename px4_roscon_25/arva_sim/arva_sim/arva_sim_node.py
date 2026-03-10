import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from pieps_interfacer.msg import PiepsMeasurements
import numpy as np
import math

class ArvaSim(Node):
    def __init__(self):
        super().__init__('arva_sim_node')

        self._drone_pos = None
        self._drone_heading = None
        self._tx_pos = None
        self._tx_theta = None

        self.initSubscribers()
        self.initPublishers()
        self.initTimers()
        self.spawnTransmitter()
        self.get_logger().info('ArvaSim node has been started.')


    def initSubscribers(self):
        # QoS profile compatible with PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self._local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.localPositionCallback,
            qos_profile
        )


    def initPublishers(self):
        self._arva_pub = self.create_publisher(PiepsMeasurements, '/pieps/measurement', 10)


    def initTimers(self):
        self._arva_timer = self.create_timer(0.25, self.arvaTimer)


    def spawnTransmitter(self):
        # TODO hardcoded for now. Use searchfield coords to randomly init location
        # Transmitter position in NED frame: [North, East, Down]
        # Here: 20 m East of the origin (same NED frame as PX4 local position)
        self._tx_pos = np.array([10.0, 30.0, 0.0])
        m_vec = np.array([0.0, 1.0, 0.0])
        self._tx_theta = np.radians(-90.0)
        
        # Yaw rotation about +Down axis in NED (clockwise-positive heading convention)
        R_tx = np.array([
            [np.cos(self._tx_theta),  np.sin(self._tx_theta), 0],
            [-np.sin(self._tx_theta), np.cos(self._tx_theta), 0],
            [0,                 0,                1],
        ])
        
        self._m_vec_wf = R_tx @ m_vec 
        self.get_logger().info(f'Transmitter spawned at: {self._tx_pos} with orientation (deg): {math.degrees(self._tx_theta)}')


    def localPositionCallback(self, msg):
        z_transform = 0.65  # TODO Create Parameter to compensate for lowering rel to base_link
        # PX4 VehicleLocalPosition is NED: x=North, y=East, z=Down
        self._drone_pos = np.array([round(msg.x,2), round(msg.y,2), round(msg.z,2) - z_transform])
        self._drone_heading = msg.heading

        # self._drone_pos = np.array([0.0, 0.0, 0 + z_transform])
        # self._drone_heading = 0.0


    def arvaTimer(self):
        if self._drone_pos is None:
            self.get_logger().warn('Waiting for drone position...', throttle_duration_sec=2.0)
            return

        distance, delta = self.computeArvaSignal()

        # Apply quantization mapping
        distance, delta = self.mapMeasuremntToPiepsMsgs(distance, delta)
        if distance >= 0:
            msg = PiepsMeasurements()
            msg.distance = float(distance)
            msg.angle = float(math.degrees(delta))

            if distance > 2.0:
                msg.angle_valid = True
            else:
                msg.angle_valid = False

            if distance > 0.0:
                msg.distance_valid = True
            else:
                msg.distance_valid = False

            self._arva_pub.publish(msg)
            # self.get_logger().info(f'Published ARVA Signal  - Distance: {distance:.2f} m, Angle: {math.degrees(delta):.2f} deg')

        else:
            self.get_logger().info(f'ARVA Signal - Out of range with {distance:.2f} m')



    def computeArvaSignal(self):
        r = self._drone_pos - self._tx_pos
        r_norm = np.linalg.norm(r)
        # NED components
        n, e, d = r[0], r[1], r[2]
        A = np.array([
            [2*n**2 - e**2 - d**2,  3*n*e,                  3*n*d],
            [3*n*e,                 2*e**2 - n**2 - d**2,   3*e*d],
            [3*n*d,                 3*e*d,                  2*d**2 - n**2 - e**2]])

        H_strength = ((1.0 / (4.0 * np.pi * r_norm**5)) * (A @ self._m_vec_wf))
        # Horizontal bearing in NED: atan2(East, North)
        H_dir = np.arctan2(H_strength[1], H_strength[0])
        # self.get_logger().info(f'Raw ARVA Signal: {H_strength} m;{math.degrees(H_dir):.2f} deg')

        H_norm = np.linalg.norm(H_strength)
        constant = 1.5420 / (4.0 * np.pi)  # Calibration constant from original code

        d3 = (constant / H_norm) ** (1/3) if H_norm > 1e-12 else -1

        d1 = (constant / abs(H_strength[0])) ** (1/3) if abs(H_strength[0]) > 1e-12 else -1

        r_noise = 15.00  # TODO make parameter. max detection range in meters

        if d3 <= 2.0:
            distance = d3
            delta = 0.0
        elif d1 <= r_noise:
            distance = d1
            # Flux lines are undirected: choose the field direction (or its anti-parallel)
            # that points into the same half-plane as the vector toward the transmitter.
            # This prevents the drone from following the flux line *away* from the beacon.
            to_tx_h = self._tx_pos[:2] - self._drone_pos[:2]  # [North, East] toward transmitter
            H_h = np.array([H_strength[0], H_strength[1]])     # horizontal field vector
            if np.dot(H_h, to_tx_h) < 0:
                H_dir_corrected = H_dir + np.pi
            else:
                H_dir_corrected = H_dir

            delta = self._drone_heading - H_dir_corrected
            # Normalize to [-pi, pi]
            delta = (delta + np.pi) % (2 * np.pi) - np.pi
        else:
            distance = -1
            delta = 0.0
        
        return distance, delta


    def mapMeasuremntToPiepsMsgs(self, distance, delta):
        """Quantize the continuous angle measurement to discrete levels."""
        angle_deg = math.degrees(delta)

        # Quantize to ceiling threshold levels
        thresholds = [20, 30, 40, 50, 60]
        abs_angle = abs(angle_deg)

        # Find the ceiling threshold
        quantized_abs = 60  # Default to max
        for threshold in thresholds:
            if abs_angle < threshold:
                quantized_abs = threshold
                break

        # Apply sign
        quantized_deg = quantized_abs if angle_deg >= 0 else -quantized_abs
        quantized_rad = math.radians(quantized_deg)

        return distance, quantized_rad


def main():
    rclpy.init()
    arva_sim_node = ArvaSim()
    rclpy.spin(arva_sim_node)
    arva_sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
