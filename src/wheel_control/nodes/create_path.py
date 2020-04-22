if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('create_path')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'f': 
                goStraight(1,control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)
                
            elif key == 't' :
                angle_degree = 90
                turn(angle_degree,control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)
            elif key == 'r' :
                drawRectangle(control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)  
                
#vaste angular snelheid <-> duurtijd => hoek
            elif key == ' ' or key == 's' :#dit in een functie steken stoppen -> hoe meerdere argumentern terug geven?
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
                controleEnPublish(control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)
            else:
                if (key == '\x03'):
                    break

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
