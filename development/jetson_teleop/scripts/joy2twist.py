#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

#Este script transforma los comandos del joystick a mensajes de tipo Twist y los publica
#Autor: Alexander Peregrina
#Axis 0: Controla los movimientos horizontales del analogo izquierdo ps3 joystick izq:1 der:-1 para controlar el steering angle
#Axis 13: Usa el boton R2 para controlar el throtle (aceleracion)

def callback(data):
    twist = Twist()
    twist.angular.z = 10*data.axes[0]
    if data.buttons[6] == 1 and data.buttons[7] == 0:
    	twist.linear.x = (5 * (data.axes[4] + 1) - 10)/8
    elif data.buttons[6] == 0 and data.buttons[7] == 1:
        #twist.linear.x = -5 * (data.axes[4]+ 1) + 10
	twist.linear.x = 1.18
	time.sleep(0.1)
	twist.linear.x = 0
	time.sleep(0.1)
    elif data.buttons[6] == 1 and data.buttons[7] == 1:
	twist.linear.x = 5 * (data.axes[4] + 1) - 10
    elif data.buttons[6] == 0 and data.buttons[7] == 0:
	twist.linear.x = (-5 * (data.axes[4]+ 1) + 10)/8
	
    pub.publish(twist)

def start():
    global pub
    #Publicando en el topico turtle1/cmd_vel para mover a la tortuga
    pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)
    #Suscrito a joystick para manipular la tortuga
    rospy.Subscriber("joy", Joy, callback)
    rospy.init_node('joy2twist', anonymous = True)
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException: 
        pass 
