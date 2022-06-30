import math
import time
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class PID:
    def __init__(self, kp=2, ki=0.0, kd=0.0, SetPoint=0.0, current_time=None):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.sample_time = 0.01
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear(SetPoint)

    def clear(self, SetPoint):
        self.SetPoint = SetPoint
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.windup_guard = 10.0
        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sample_time:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time


class MyRobot1(RCJSoccerRobot):
    def run(self):
        control = PID(15, 0.1, 0.1, -0.4)
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

                control.update(robot_pos[1])
                print(robot_pos[1])

                self.left_motor.setVelocity(-control.output)
                self.right_motor.setVelocity(-control.output)
