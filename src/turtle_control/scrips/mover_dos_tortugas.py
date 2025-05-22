#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def mover_dos_tortugas():
    rospy.init_node('mover_dos_tortugas', anonymous=True)

    # Publicadores para ambas tortugas
    pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/Tortuga2/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    inicio = rospy.get_time()

    while not rospy.is_shutdown():
        tiempo = (rospy.get_time() - inicio) % 4  # ciclo de 4 segundos

        cmd = Twist()
        if tiempo < 2:
            # Fase de avance
            cmd.linear.x = 2.0
            cmd.angular.z = 0.0
        else:
            # Fase de giro
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5

        pub1.publish(cmd)
        pub2.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        mover_dos_tortugas()
    except rospy.ROSInterruptException:
        pass
