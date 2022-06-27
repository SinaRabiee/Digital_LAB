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

                distance = ball_data["strength"]
                theta = ball_data["direction"][1]
                FrontBack = ball_data["direction"][0]
                kmax = 150
                kmin = 15
                kt = 10

                print(ball_data)
                if abs(theta) > 0.05:
                     self.right_motor.setVelocity(-kt*theta)
                     self.left_motor.setVelocity(kt*theta) 
                elif FrontBack < -0.01:
                    self.right_motor.setVelocity(-9)
                    self.left_motor.setVelocity(9) 
                elif distance > 15:
                     self.right_motor.setVelocity(kmax/distance)
                     self.left_motor.setVelocity(kmax/distance) 
                else:
                     self.right_motor.setVelocity(kmin/distance)
                     self.left_motor.setVelocity(kmin/distance) 
                 
             
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
