#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import pygame
import time

rospy.init_node('manual_control')

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
cmd_vel_pub1 = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=1)
cmd_vel_pub2 = rospy.Publisher('robot2/cmd_vel', Twist, queue_size=1)
cmd_vel_pub3 = rospy.Publisher('robot3/cmd_vel', Twist, queue_size=1)
cmd_vel_pub4 = rospy.Publisher('robot4/cmd_vel', Twist, queue_size=1)
cmd_vel_pub5 = rospy.Publisher('robot5/cmd_vel', Twist, queue_size=1)

pub = cmd_vel_pub

pygame.init()

window_size = (100, 100)
screen = pygame.display.set_mode(window_size)
pygame.display.set_caption("Manual Control")

linear_speed = 0.22
angular_speed = 1
cmd = Twist()

running = True

start_time = time.time()
key_press_start_time = None
total_key_press_time = 0.0
pressed_keys = set()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key not in pressed_keys:
                pressed_keys.add(event.key)
                if len(pressed_keys) == 1:
                    key_press_start_time = time.time()

            if event.key == pygame.K_UP:
                # Mover para a frente
                cmd.linear.x = linear_speed
            elif event.key == pygame.K_DOWN:
                # Mover para trás
                cmd.linear.x = -linear_speed
            elif event.key == pygame.K_LEFT:
                # Girar para a esquerda
                cmd.angular.z = angular_speed
            elif event.key == pygame.K_RIGHT:
                # Girar para a direita
                cmd.angular.z = -angular_speed
            elif event.key == pygame.K_0:
                rospy.loginfo('Publishing on default robot')
                pub = cmd_vel_pub
            elif event.key == pygame.K_1:
                rospy.loginfo('Publishing on robot 1')
                pub = cmd_vel_pub1
            elif event.key == pygame.K_2:
                rospy.loginfo('Publishing on robot 2')
                pub = cmd_vel_pub2
            elif event.key == pygame.K_3:
                rospy.loginfo('Publishing on robot 3')
                pub = cmd_vel_pub3
            elif event.key == pygame.K_4:
                rospy.loginfo('Publishing on robot 4')
                pub = cmd_vel_pub4
            elif event.key == pygame.K_5:
                rospy.loginfo('Publishing on robot 5')
                pub = cmd_vel_pub5
            elif event.key == pygame.K_ESCAPE:
                running = False
        elif event.type == pygame.KEYUP:
            if event.key in pressed_keys:
                pressed_keys.remove(event.key)
                if len(pressed_keys) == 0:
                    key_press_end_time = time.time()
                    total_key_press_time += key_press_end_time - key_press_start_time
                    key_press_start_time = None

    cmd.linear.x = 0.0
    cmd.angular.z = 0.0

    if pygame.K_UP in pressed_keys:
        cmd.linear.x = linear_speed
    if pygame.K_DOWN in pressed_keys:
        cmd.linear.x = -linear_speed
    if pygame.K_LEFT in pressed_keys:
        cmd.angular.z = angular_speed
    if pygame.K_RIGHT in pressed_keys:
        cmd.angular.z = -angular_speed

    pub.publish(cmd)

end_time = time.time()
total_execution_time = end_time - start_time

cmd.linear.x = 0.0
cmd.angular.z = 0.0
pub.publish(cmd)

pygame.quit()

print("\033[92mTempo total de execução:", round(total_execution_time, 2), "segundos\033[0m")
print("\033[92mTempo total que o usuário precisou apertar teclas:", round(total_key_press_time, 2), "segundos\033[0m")
