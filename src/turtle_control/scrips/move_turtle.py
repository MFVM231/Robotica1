#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def mover_tortuga():
    rospy.init_node('mover_tortuga', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    inicio = rospy.get_time()

    while not rospy.is_shutdown():
        tiempo_actual = rospy.get_time()
        tiempo_transcurrido = (tiempo_actual - inicio) % 4  # ciclo de 4 segundos

        cmd = Twist()

        if tiempo_transcurrido < 2:
            # Fase de avance (2 segundos)
            cmd.linear.x = 2.0
            cmd.angular.z = 0.0
        else:
            # Fase de giro (2 segundos)
            cmd.linear.x = 0.5
            cmd.angular.z = 1.5

        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        mover_tortuga()
    except rospy.ROSInterruptException:
        pass