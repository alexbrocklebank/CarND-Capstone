from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
        accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel,
        max_steer_angle, min_speed):

        # Class member variables
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.min_speed = min_speed
        self.last_time = rospy.get_time()

        # STEERING YAW CONTROLLER
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed,
                                            max_lat_accel, max_steer_angle)

        # THROTTLE PID CONTROLLER
        kp = 0.35
        ki = 0.0
        kd = 0.0
        mn = 0      # minimum throttle value
        mx = 1.0   # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # VELOCITY LOW PASS FILTER
        tau = 0.5   # 1/(2pi*tau) = cutoff frequency
        ts = 0.02   # sample time
        self.vel_lpf = LowPassFilter(tau, ts)


    def control(self, dbw_enabled, current_vel, linear_vel,
                curr_ang_vel, angular_vel):
        """Determine the Throttle, Steering, and Brake values to update the
            drive-by-wire (DBW) Node.

        Args:
            dbw_enabled (Bool): whether the system under auto or manual control
            current_vel (float): current velocity value
            linear_vel (float): linear velocity desired value
            curr_ang_vel (float): current vehicle angular velocity value
            angular_vel (float): angular velocity desired value

        Returns:
            float: new Throttle value determined by controller
            float: new Brake value determined by controller
            float: new Steering value determined by controller
        """

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel,
                                                    current_vel)
        # Debugging steering equation
        #steering = angular_vel * self.steer_ratio

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        # Get the time elapsed between samples
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        brake = 0.
        if throttle < 0.:
            brake = -throttle

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400 # N*m - to hold the car in place if we are stopped

        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius  #Torque N*m

        return throttle, brake, steering
