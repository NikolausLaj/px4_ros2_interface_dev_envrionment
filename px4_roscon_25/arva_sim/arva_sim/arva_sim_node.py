import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from pieps_interfacer.msg import PiepsMeasurements
import numpy as np
import math

class ArvaSim(Node):
    def __init__(self):
        super().__init__('arva_sim_node')
        
        # Initialize state variables
        self._drone_pos = None
        self._drone_quat = np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion
        
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
        
        self._attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitudeCallback,
            qos_profile
        )
        
    def initPublishers(self):
        self._arva_pub = self.create_publisher(PiepsMeasurements, '/pieps/measurement', 10) 
        
    def initTimers(self):
        self._arva_timer = self.create_timer(0.25, self.arvaTimer)
    
    
    def spawnTransmitter(self):
        # TODO hardcoded for now. Use searchfield coords to randomly init location
        # Transmitter position in NED frame
        self._tx_pos = np.array([10.0, 10.0, 0.0])  # 10m North, 5m East, on ground
        self._tx_quat = None  # Default orientation (dipole along X)
        self.get_logger().info(f'Transmitter spawned at: {self._tx_pos}')
        
        
    def localPositionCallback(self, msg):
        z_transform = 0.65  # TODO Create Parameter to compensate for lowering rel to base_link
        self._drone_pos = np.array([msg.x, msg.y, msg.z + z_transform])
        
        
    def attitudeCallback(self, msg):
        # PX4 quaternion is [w, x, y, z], we need [x, y, z, w]
        self._drone_quat = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
    
    
    def arvaTimer(self):
        if self._drone_pos is None:
            self.get_logger().warn('Waiting for drone position...', throttle_duration_sec=2.0)
            return
            
        distance, delta = self.computeArvaSignal()
        
        # Apply quantization mapping
        distance, delta_quantized = self.mapMeasuremntToPiepsMsgs(distance, delta)
        
        if distance >= 0:
            msg = PiepsMeasurements()
            msg.distance = float(distance)
            msg.angle = float(delta_quantized)
            
            if distance > 2.0:
                msg.angle_valid = True
            else:
                msg.angle_valid = False
                
            if distance > 0.0:
                msg.distance_valid = True
            else:
                msg.distance_valid = False
                
            self._arva_pub.publish(msg)
            
        else:
            self.get_logger().info('ARVA Signal - Out of range')
        
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
        
        
    def computeArvaSignal(self):
        # Transmitter magnetic moment (dipole along X-axis)
        m_vec = np.array([1.0, 0.0, 0.0])
        
        # Rotate magnetic moment by transmitter orientation if specified
        if self._tx_quat is not None:
            Rt = self.quaternion2RotationMatrix(self._tx_quat)
            m_vec = Rt @ m_vec
        
        # Vector from transmitter to receiver
        r = self._drone_pos - self._tx_pos
        r_norm = np.linalg.norm(r)
        
        if r_norm < 1e-6:
            return 0.0, 0.0, np.array([0.0, 0.0, 0.0])
        
        # Build the magnetic dipole matrix A
        x, y, z = r[0], r[1], r[2]
        A = np.array([
            [2*x**2 - y**2 - z**2,  3*x*y,                  3*x*z],
            [3*x*y,                 2*y**2 - x**2 - z**2,   3*y*z],
            [3*x*z,                 3*y*z,                  2*z**2 - x**2 - y**2]
        ])
        
        # Magnetic field H = (1 / 4π r^5) * A * m
        H = (1.0 / (4.0 * np.pi * r_norm**5)) * (A @ m_vec)
        
        # Transform to receiver body frame
        R_drone = self.quaternion2RotationMatrix(self._drone_quat)
        Hb = R_drone @ H  # H in body frame
        
        # Distance estimation (inverting the 1/r^3 relationship)
        H_norm = np.linalg.norm(Hb)
        constant = 1.5420 / (4.0 * np.pi)  # Calibration constant from original code
        
        # 3-axis distance estimate (better for close range)
        d3 = (constant / H_norm) ** (1/3) if H_norm > 1e-12 else -1
        
        # Single-axis distance estimate (better for far range)
        d1 = (constant / abs(Hb[0])) ** (1/3) if abs(Hb[0]) > 1e-12 else -1
        
        # Range logic from original code
        r_noise = 50.0  # max detection range in meters
        
        if d3 <= 2.0:
            distance = d3
            delta = 0.0
        elif d1 <= r_noise:
            distance = d1
            delta = np.arctan2(Hb[1], Hb[0])
        else:
            distance = -1
            delta = 0.0
        
        return distance, delta
        
        
    def quaternion2RotationMatrix(self, q):
        """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
        x, y, z, w = q[0], q[1], q[2], q[3]
        
        # Normalize
        n = np.sqrt(x*x + y*y + z*z + w*w)
        if n < 1e-12:
            return np.eye(3)
        x, y, z, w = x/n, y/n, z/n, w/n
        
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        
def main():
    rclpy.init()
    arva_sim_node = ArvaSim()
    rclpy.spin(arva_sim_node)
    arva_sim_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
