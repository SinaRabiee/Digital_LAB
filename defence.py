import math
import utils
import time
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class PID:
    def __init__(
        self,
        kp=10,
        kd=0,
        ki=0,
        set_point=0.01,
        current_time=None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.sampling_time = 0.1
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear(set_point)

    def clear(self, set_point):
        self.SetPoint = set_point
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        # saturation bound for motor velocity
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
        control_theta = PID(2, 0, 0)
        control_balldis = PID(1, 0, 0)
        control_dis = PID(8, 0, 1)
        control_ang = PID(5, 0, 0)
        L = 0.08
        R = 0.02
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # self.left_motor.setVelocity(0)
                    # self.right_motor.setVelocity(0)
                    continue

                heading = self.get_compass_heading()
                robot_pos = self.get_gps_coordinates()
                sonar_values = self.get_sonar_values()
                direction = utils.get_direction(ball_data["direction"])

                xd = 0.75
                yd = 0.0
                x = robot_pos[1]
                y = robot_pos[0]
                balldis = ball_data["strength"]
                balltheta = ball_data["direction"][1]

                if y > 0:
                    ang = -math.pi + heading - math.atan2(yd - y, -xd + x)
                else:
                    ang = math.pi + heading - math.atan2(yd - y, -xd + x)
                dis = math.sqrt((yd - y) ** 2 + (xd - x) ** 2)
                control_dis.update(dis)
                control_ang.update(ang)
                control_theta.update(balltheta)
                control_balldis.update(-1 / balldis)
                if abs(ang) > 0.1:
                    w = control_ang.output
                    self.left_motor.setVelocity(w)
                    self.right_motor.setVelocity(-w)
                elif dis > 0.1:
                    v = control_dis.output
                    self.left_motor.setVelocity(v)
                    self.right_motor.setVelocity(v)
                    print(f"dis : {dis}")
                else:
                    print(balldis)
                    if balldis > 100:
                        print(ball_data)
                        if abs(balltheta) > 0.05:
                            self.left_motor.setVelocity(-10)
                            self.right_motor.setVelocity(10)
                        else:
                            self.left_motor.setVelocity(10)
                            self.right_motor.setVelocity(10)
                    else:
                        self.left_motor.setVelocity(-5)
                        self.right_motor.setVelocity(5)
