from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity,brake_deadband,decel_limit,accel_limit,
    			wheel_radius,wheel_base,steer_ratio,max_lat_accel,max_steer_angle,min_speed):
        # TODO: Implement
        self.steer_controller = YawController(wheel_base,steer_ratio,min_speed,max_lat_accel,max_steer_angle)
        self.steer_filter = LowPassFilter(1,2)
        self.speed_controller = PID(1,0,0,mn=-1,mx=1)
        self.speed_filter = LowPassFilter(1,1)


    def control(self, target_velocity, target_angular_velocity, current_velocity, current_angular_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer_angle = self.steer_controller.get_steering(target_velocity, target_angular_velocity, current_velocity)
        steer_angle = self.steer_filter.filt(steer_angle)

        speed_err = target_velocity - current_velocity
        accel = self.speed_controller.step(speed_err,0.02)
        accel = self.speed_filter.filt(accel)

        if accel >= 0:
        	throttle = accel
        	brake = 0
        else:
        	throttle = 0
        	brake = -3000*accel

        return throttle, brake, steer_angle
