import math
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
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

                yd = ball_data["direction"][1]
                xd = ball_data["direction"][0]
                ex = xd - robot_pos[1]
                ey = yd - robot_pos[0]
                kpx = 2
                kpy = 10

                if (abs(ex) > 1e-2):
                    self.left_motor.setVelocity(kpx*ex)
                    self.right_motor.setVelocity(kpx*ex)
                else:
                    self.left_motor.setVelocity(5)
                    if (yd > 0):
                        if (heading - math.pi/2 > 1e-2):
                            self.left_motor.setVelocity(kpy*ey)
                            self.right_motor.setVelocity(kpy*ey)
                        else:
                            self.left_motor.setVelocity(7)
                            self.right_motor.setVelocity(0)

                    else:
                        if (heading + math.pi/2 > 1e-2):
                            self.left_motor.setVelocity(kpy*ey)
                            self.right_motor.setVelocity(kpy*ey)
                        else:
                            self.left_motor.setVelocity(0)
                            self.right_motor.setVelocity(7)

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
