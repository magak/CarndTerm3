import rospy

from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_speed, max_lat_accel, max_steer_angle
			,vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius):

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

	self.vehicle_mass = vehicle_mass
	self.fuel_capacity = fuel_capacity
	self.brake_deadband = brake_deadband
	self.decel_limit = decel_limit
	self.accel_limit = accel_limit
	self.wheel_radius = wheel_radius

	self.low_pass_steer_filter = LowPassFilter(0.6, 1.0)
	self.low_pass_accel_filter = LowPassFilter(0.6, 1.0)
	self.low_pass_throttle_filter = LowPassFilter(0.6, 1.0)

	self.acceleration_controller  = PID(0.3, 5.0, 0.0003, self.decel_limit, self.accel_limit)
	self.throttle_controller = PID(0.3, 5.0, 0.0003, 0.0, 1.0)	
	
	self.throttle_controller.reset()
	self.acceleration_controller.reset()
	
	self.lastControlTime = None
	self.lastVelocity = 0.0

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_status):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	
	steering = self.yaw_controller.get_steering(linear_velocity, self.low_pass_steer_filter.filt(angular_velocity), current_velocity)

	current_time = time.time()
	if self.lastControlTime is None:
		sample_time = 0.01
	else:
		sample_time = current_time - self.lastControlTime
	
	self.lastControlTime = current_time
	
	throttle = 0.0
	brake = 0.0

	acceleration = self.low_pass_accel_filter.filt(self.acceleration_controller.step((linear_velocity-current_velocity)/sample_time, sample_time))

	rospy.loginfo(str(acceleration))
	
	if(acceleration < 0.0):
		# braking
		if(abs(acceleration) > self.brake_deadband):
			brake = (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY)*abs(acceleration)*self.wheel_radius
			#rospy.loginfo("Braking:")
		#else:
			#rospy.loginfo("Brake deadband:")
	else:
		# accelerating
		currentAcceleration =  (current_velocity-self.lastVelocity)/sample_time
		throttle = self.low_pass_throttle_filter.filt(self.throttle_controller.step(acceleration-currentAcceleration, sample_time))
		#rospy.loginfo("Throttle:")
	
	self.lastVelocity = current_velocity
	
	#throttle = self.throttle_controller.step(linear_velocity-current_velocity, sample_time)
	#rospy.loginfo("throttle= "+str(throttle)+" brake= "+str(brake)+" steering="+str(steering))

        return throttle, brake, steering
