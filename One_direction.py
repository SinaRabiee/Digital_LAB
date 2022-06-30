import math 
import utils
import time
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class PID:
    def __init__(self, kp=10, kd=0, ki=0, sampling_time=0.1, set_point=0.01, current_time = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        errori = 0
        previous_error = 0
        self.sampling_time = sampling_time
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        
        self.clear(set_point)
           
    def clear(self,set_point):
        self.SetPoint = set_point
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        #saturation bound for motor velocity
        self.windup = 10.0
        self.output = 0.0
        
    def update(self, feedback_value, current_time=None):
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sampling_time:
            self.PTerm = self.kp * error
            self.ITerm += error * delta_time

            if self.ITerm < -self.windup:
                self.ITerm = -self.windup
            elif self.ITerm > self.windup:
                self.ITerm = self.windup
            
            self.DTerm = 0.0
            self.DTerm = delta_error / delta_time
            
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.ki * self.ITerm) + (self.kd * self.DTerm)

class MyRobot1(RCJSoccerRobot):
    def run(self):
        control = PID(15,0.1,0.1,0.1,-0.4)
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data() 
                while self.is_new_team_data():
                    team_data = self.get_new_team_data() 

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                heading = self.get_compass_heading() 
                robot_pos = self.get_gps_coordinates()                              
                sonar_values = self.get_sonar_values()  
                
                control.update(robot_pos[1])
                print(robot_pos[1])
                

                self.left_motor.setVelocity(-control.output)
                self.right_motor.setVelocity(-control.output)
		 
                '''# Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])
                if direction == 0:
                    left_speed = 7
                    right_speed = 7
                else:
                    left_speed = direction * 4
                    right_speed = direction * -4

                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
                self.send_data_to_team(self.player_id)'''

