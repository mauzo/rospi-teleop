from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import rospy

linV = 0.4
angV = 2.0

def wait_for_subscribers(pub):
    i = 0
    while not rospy.is_shutdown() and pub.get_num_connections() == 0:
        if i == 4:
            print("Waiting for subscriber to connect to {}".format(pub.name))
        rospy.sleep(0.5)
        i += 1
        i = i % 5
    if rospy.is_shutdown():
        raise Exception("Got shutdown request before subscribers connected")

def convert_demand(action):
    cmd = Twist()
    x = 0.0
    phi = 0.0

    if action == "forward":
        x = linV
    elif action == "backward":
        x = -linV
    elif action == "right":
        phi = -angV
        x = linV
    elif action == "left":
        phi = angV
        x = linV

    cmd.linear = Vector3(x, 0, 0)
    cmd.angular = Vector3(0, 0, phi)

    return cmd

def run (win):
    win.setup()

    buggy_cmd = Twist()

    pub = rospy.Publisher('demand_out', Twist, queue_size=1)
    rospy.init_node('pub')

    rate = rospy.Rate(5)
    state = 'stop'

    try:
        wait_for_subscribers(pub)

        running = True

        while not rospy.is_shutdown() and running:

            for action in win.get_actions():

                if action == "quit":
                    state   = "stop"
                    print("Exiting program")
                    running = False

                elif action == "faster":
                    linV += 0.1

                elif action == "slower":
                    linV -= 0.1

                elif action == "turn-faster":
                    angV += 0.5

                elif action == "turn-slower":
                    angV -= 0.5

                else:
                    state = action

            win.update()
            buggy_cmd = convert_demand(state)
            pub.publish(buggy_cmd)
            rate.sleep()
    finally:
        win.quit()
        pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

