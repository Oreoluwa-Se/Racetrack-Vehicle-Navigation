#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
	def __init__(self, waypoints):
		self.vars                = cutils.CUtils()
		self._current_x          = 0
		self._current_y          = 0
		self._current_yaw        = 0
		self._current_speed      = 0
		self._desired_speed      = 0
		self._current_frame      = 0
		self._current_timestamp  = 0
		self._start_control_loop = False
		self._set_throttle       = 0
		self._set_brake          = 0
		self._set_steer          = 0
		self._waypoints          = waypoints
		self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
		self._pi                 = np.pi
		self._2pi                = 2.0 * np.pi

	def update_values(self, x, y, yaw, speed, timestamp, frame):
		self._current_x         = x
		self._current_y         = y
		self._current_yaw       = yaw
		self._current_speed     = speed
		self._current_timestamp = timestamp
		self._current_frame     = frame
		if self._current_frame:
			self._start_control_loop = True

	def update_desired_speed(self):
		min_idx       = 0
		min_dist      = float("inf")
		desired_speed = 0
		for i in range(len(self._waypoints)):
			dist = np.linalg.norm(np.array([
					self._waypoints[i][0] - self._current_x,
					self._waypoints[i][1] - self._current_y]))
			if dist < min_dist:
				min_dist = dist
				min_idx = i
		if min_idx < len(self._waypoints)-1:
			desired_speed = self._waypoints[min_idx][2]
		else:
			desired_speed = self._waypoints[-1][2]
		self._desired_speed = desired_speed

	def update_waypoints(self, new_waypoints):
		self._waypoints = new_waypoints

	def get_commands(self):
		return self._set_throttle, self._set_steer, self._set_brake

	def set_throttle(self, input_throttle):
		# Clamp the throttle command to valid bounds
		throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
		self._set_throttle = throttle

	def set_steer(self, input_steer_in_rad):
		# Covnert radians to [-1, 1]
		input_steer = self._conv_rad_to_steer * input_steer_in_rad

		# Clamp the steering command to valid bounds
		steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
		self._set_steer = steer

	def set_brake(self, input_brake):
		# Clamp the steering command to valid bounds
		brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
		self._set_brake = brake

	def update_controls(self):
		######################################################
		# RETRIEVE SIMULATOR FEEDBACK
		######################################################
		x               = self._current_x
		y               = self._current_y
		yaw             = self._current_yaw
		v               = self._current_speed
		self.update_desired_speed()
		v_desired       = self._desired_speed
		t               = self._current_timestamp
		waypoints       = self._waypoints
		throttle_output = 0
		steer_output    = 0
		brake_output    = 0

		######################################################
		######################################################
		# MODULE 7: DECLARE USAGE VARIABLES HERE
		######################################################
		######################################################
		"""
			Use 'self.vars.create_var(<variable name>, <default value>)'
			to create a persistent variable (not destroyed at each iteration).
			This means that the value can be stored for use in the next
			iteration of the control loop.

			Example: Creation of 'v_previous', default value to be 0
			self.vars.create_var('v_previous', 0.0)

			Example: Setting 'v_previous' to be 1.0
			self.vars.v_previous = 1.0

			Example: Accessing the value from 'v_previous' to be used
			throttle_output = 0.5 * self.vars.v_previous
		"""

		# creating vatiables for storage and access
		self.vars.create_var('v_previous', 0.0)
		self.vars.create_var('t_previous', 0.0)
		self.vars.create_var('int_err_previous', 0.0)
		self.vars.create_var('throt_previous', 0.0) # same as previous acceleration
		
		# initialize parameters
		# Parameters
		KP_throttle = 3 
		KI_throttle = 0.5 * KP_throttle
		KD_throttle = 0.5 * KI_throttle

		# Skip the first frame to store previous values properly
		if self._start_control_loop:
			"""
				Controller iteration code block.

				Controller Feedback Variables:
					x               : Current X position (meters)
					y               : Current Y position (meters)
					yaw             : Current yaw pose (radians)
					v               : Current forward speed (meters per second)
					t               : Current time (seconds)
					v_desired       : Current desired speed (meters per second)
									  (Computed as the speed to track at the
									  closest waypoint to the vehicle.)
					waypoints       : Current waypoints to track
									  (Includes speed to track at each x,y
									  location.)
									  Format: [[x0, y0, v0],
											   [x1, y1, v1],
											   ...
											   [xn, yn, vn]]
									  Example:
										  waypoints[2][1]: 
										  Returns the 3rd waypoint's y position

										  waypoints[5]:
										  Returns [x5, y5, v5] (6th waypoint)
				
				Controller Output Variables:
					throttle_output : Throttle output (0 to 1)
					steer_output    : Steer output (-1.22 rad to 1.22 rad)
					brake_output    : Brake output (0 to 1)
			"""

			######################################################
			######################################################
			# MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
			######################################################
			######################################################
			"""
				Implement a longitudinal controller here. Remember that you can
				access the persistent variables declared above here. For
				example, can treat self.vars.v_previous like a "global variable".
			"""
			
			# Change these outputs with the longitudinal controller. Note that
			# brake_output is optional and is not required to pass the
			# assignment, as the car will naturally slow down over time.
			error = v_desired - v
			output = self.long_control([KP_throttle, KI_throttle, KD_throttle], error, t)
			
			if (output >= 0):
				throttle_output = output
				brake_output = 0
			else:
				throttle_output = 0
				brake_output    = -output

			######################################################
			######################################################
			# MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
			######################################################
			######################################################
			"""
				Implement a lateral controller here. Remember that you can
				access the persistent variables declared above here. For
				example, can treat self.vars.v_previous like a "global variable".
			"""
			
			KP_steer = 0.8
			# Change the steer output with the lateral controller. 
			steer_output    = self.lat_control(KP_steer, waypoints, yaw, x, y, v) 

			######################################################
			# SET CONTROLS OUTPUT
			######################################################
			self.set_throttle(throttle_output)  # in percent (0 to 1)
			self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
			self.set_brake(brake_output)        # in percent (0 to 1)

		######################################################
		######################################################
		# MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
		######################################################
		######################################################
		"""
			Use this block to store old values (for example, we can store the
			current x, y, and yaw values here using persistent variables for use
			in the next iteration)
		"""
		self.vars.v_previous = v  # Store forward speed to be used in next step
		self.vars.t_previous = t
		self.vars.throt_previous = self._set_throttle

	def long_control(self, pid, v_err, time):
		
		# calculate the time differntial
		dt = self._current_timestamp - time

		""" PID equation
			Here we assume the acceleration is the same as throttle output 
			as we don't have a way of representing the engine map
			acc = KP(error) + KI (error)*dt|t,0 + Kd *d*(error)/dt
		"""  	

		# proportional
		P_throttle = pid[0] * v_err

		# integral
		v_err_tot = self.vars.int_err_previous + v_err * dt 
		self.vars.int_err_previous =v_err_tot 	
		I_throttle = pid[1] * v_err_tot

		# derivative
		D_throttle = (pid[2] * v_err/dt) if dt > 0 else 0

		# return sum
		return sum([P_throttle, I_throttle, D_throttle])

	def lat_control(self, p, waypoints, yaw, x, y, v):

		min_error = np.inf

		# Heading error -> uses two furthest points
		yaw_path = np.arctan2(waypoints[-1][1] - waypoints[0][1],
			waypoints[-1][0] - waypoints[0][0])

		# bound between pi and - pi
		while yaw_path > self._pi:
			yaw_path -= self._2pi
		# bound between pi and - pi
		while yaw_path < -self._pi:
			yaw_path += self._2pi
		
		# find closest point to the front axle center 
		for index in range(len(waypoints) - 1):

			dx = waypoints[index + 1][0] - waypoints[index][0]
			dy = waypoints[index + 1][1] - waypoints[index][1]

			# find the slope
			err = ((dy * x) - (dx * y) + (waypoints[index + 1][0]*waypoints[index][1] - waypoints[index + 1][1]*waypoints[index][0]))/np.sqrt(dy**2 + dx**2) 

			if abs(err) < abs(min_error):
				min_error = err
		
		# cross track steering
		ster_cross = np.arctan(p*min_error / v)

		# bound between pi and - pi
		while ster_cross > self._pi:
			ster_cross -= self._2pi
		# bound between pi and - pi
		while ster_cross < -self._pi:
			ster_cross += self._2pi


		# yaw heading difference
		yaw_diff = yaw_path - yaw
		print("yaw_diff  = {}".format(yaw_diff))

		
		# steering input
		ster_tot = yaw_diff + ster_cross
		while (ster_tot > self._pi):
			ster_tot -= self._2pi	

		while (ster_tot < -self._pi):
			ster_tot += self._2pi

		print("total_input = {}".format(ster_tot))

		ster_tot = max(-1.22, min(ster_tot, 1.22))

		# calculate the cross track error
		return ster_tot









