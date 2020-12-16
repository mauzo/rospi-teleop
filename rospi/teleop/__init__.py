from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import rospy

# Grr python2 rubbish
__metaclass__ = type

# An Action is a command sent from a control interface
class Action:
    __slots__   = ["type"]

    def __init__ (self, type):
        self.type   = type

# A ControlAction is an adjustment to a ControlPoint (see below). This
# adjustment can be absolute or relative and is scaled to the range 
# [-1, 1].
class ControlAction (Action):
    __slots__   = ["value", "absolute"]

    def __init__ (self, value, absolute=False, **kw):
        super(ControlAction, self).__init__(**kw)

        self.value      = value
        self.absolute   = absolute

def clamp (v, a, b):
    if v < a:
        return a
    if v > b:
        return b
    return v

# A ControlPoint is a value that can be changed by a control interface.
# It can take positive or negative values up to a certain maximum.
class ControlPoint:
    __slots__ = [
        "value",    # The current value of the control point
        "step",     # The size of a relative change
        "adj",      # The steps to change .step by
        "max",      # The maximim allowed value
    ]

    def __init__ (self, value, step, adjust, max):
        self.value  = value
        self.step   = step
        self.adj    = adjust
        self.max    = max

    # Use a control message to change the current value.
    def apply (self, control):
        if control.absolute:
            self.value = control.value * self.max
        else:
            v           = self.value + control.value * self.step
            self.value  = clamp(v, -self.max, self.max)

    # Use a control message to change the current step size.
    def adjust (self, control):
        if control.absolute:
            self.step   = control.value * self.max
        else:
            s           = self.step + control.value * self.adj
            self.step = clamp(s, 0, self.max)

    # Return a scaled value
    def scale (self, sc):
        return (self.value / self.max) * sc

    def scale_step (self, sc):
        return (self.step / self.max) * sc

class TwistController:
    __slots__ = ["speed", "turn"]

    def __init__ (self):
        # XXX These need adjusting. The numbers are:
        #   initial value
        #   step applied by e.g. pressing W
        #   adjustment to step e.g. by pressing arrows
        #   max value
        # 2.0 seems rather high for turn?
        self.speed  = ControlPoint(0, 0.4, 0.1, 4.0)
        self.turn   = ControlPoint(0, 0.5, 0.1, 2.0)

    def stop (self):
        self.speed.value    = 0
        self.turn.value     = 0

    def handle_action (self, action):
        if action.type == "stop":
            self.stop()

        elif action.type == "speed":
            self.speed.apply(action)

        elif action.type == "turn":
            self.turn.apply(action)

        elif action.type == "adj-speed":
            self.speed.adjust(action)

        elif action.type == "adj-turn":
            self.turn.adjust(action)

    def to_twist (self):
        return Twist(
            linear=Vector3(self.speed.value, 0, 0),
            angular=Vector3(0, 0, self.turn.value),
        )

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

def run (win):
    win.setup()

    pub = rospy.Publisher('demand_out', Twist, queue_size=1)
    rospy.init_node('pub')

    rate    = rospy.Rate(5)
    state   = TwistController()

    try:
        wait_for_subscribers(pub)

        running = True

        while not rospy.is_shutdown() and running:

            for action in win.get_actions():
                if action.type == "quit":
                    state.stop()
                    print("Exiting program")
                    running = False
                else:
                    state.handle_action(action)

            win.draw(state)

            win.update()
            pub.publish(state.to_twist())
            rate.sleep()
    finally:
        win.quit()
        pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

