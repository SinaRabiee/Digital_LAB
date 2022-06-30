# rcj_soccer_player controller - ROBOT B2

# Feel free to import built-in libraries
import math  # noqa: F401
import time
from decimal import Decimal
# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class PID:
    def __init__(self, kp=10, kd=0, ki=0, sampling_time=0.1, current_time = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        errori = 0
        previous_error = 0
        self.sampling_time = sampling_time
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        
        self.clear()
           
    def clear(self):
        self.SetPoint = 0.01
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
           
            self.DTerm = delta_error / delta_time
            
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.ki * self.ITerm) + (self.kd * self.DTerm)
        
class MyRobot1(RCJSoccerRobot):
    def run(self):
        control_th = PID(2,0,0)
        control_dis = PID(1,0,0)
        control_fb = PID()
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                heading = self.get_compass_heading()  # noqa: F841
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                direction = utils.get_direction(ball_data["direction"])

                
                distance = ball_data["strength"]
                theta = ball_data["direction"][1]
                FrontBack = ball_data["direction"][0]
          
                control_th.update(theta)
                control_dis.update(-1/distance)
                print(ball_data)
                if FrontBack < -0.8 :
                    self.left_motor.setVelocity(-9)
                    self.right_motor.setVelocity(9)
                else:                  
                    vl_theta = (2*control_dis.output -0.08*control_th.output)/(2*0.02)
                    vr_theta = (2*control_dis.output + 0.08*control_th.output)/(2*0.02)
                    self.left_motor.setVelocity(vl_theta)
                    self.right_motor.setVelocity(vr_theta)

