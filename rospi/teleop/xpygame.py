import  pygame
from    rospi.teleop    import Action, ControlAction

# Grr python2 rubbish
__metaclass__ = type

DEADZONE    = 0.05

class JoyAxis:
    __slots__   = ["value", "action", "scale"]

    def __init__ (self, action, scale=1.0):
        self.value  = 0.0
        self.action = action
        self.scale  = scale

    def handle_value (self, value):
        if value < DEADZONE and value > -DEADZONE:
            value   = 0.0
        else:
            value   = round(value * self.scale, 4)

        if value == self.value:
            return []
        self.value  = value

        return [ControlAction(
            type=self.action,
            value=value, 
            absolute=True)]

# This implements the pygame interface. This can handle key-up events,
# so the motors stop as soon as you release the key. It create a tiny
# window just to accept events.
class PygameInterface:
    keydown = {
        pygame.K_ESCAPE:        Action(type="quit"),
        pygame.K_SPACE:         Action(type="stop"),
        pygame.K_w:             ControlAction(type="speed", value=1),
        pygame.K_s:             ControlAction(type="speed", value=-1),
        pygame.K_a:             ControlAction(type="turn", value=-1),
        pygame.K_d:             ControlAction(type="turn", value=1),
        pygame.K_UP:            ControlAction(type="adj-speed", value=1),
        pygame.K_DOWN:          ControlAction(type="adj-speed", value=-1),
        pygame.K_RIGHT:         ControlAction(type="adj-turn", value=1),
        pygame.K_LEFT:          ControlAction(type="adj-turn", value=-1),
    }

    keyup = {
        pygame.K_w:     ControlAction(type="speed", value=-1),
        pygame.K_s:     ControlAction(type="speed", value=1),
        pygame.K_a:     ControlAction(type="turn", value=1),
        pygame.K_d:     ControlAction(type="turn", value=-1)
    }

    joy_axis = {
        0:      JoyAxis(action="turn", scale=1.0),
        1:      JoyAxis(action="speed", scale=-1.0),
    }

    def __init__ (self):
        self.win        = None
        self.joy        = None
        self.joy_active = False

    def setup (self):
        self.setup_display()
        self.setup_joystick()

    def setup_display (self):
        pygame.display.init()
        self.win = pygame.display.set_mode((100, 100))
        pygame.key.set_repeat(0)

    def setup_joystick (self):
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            return

        self.joy    = pygame.joystick.Joystick(0)
        self.joy.init()

    def quit (self):
        pygame.quit()

    def handle_joy_event (self, event, actions):
        if event.axis not in self.joy_axis:
            return
        axis    = self.joy_axis[event.axis]
        value   = event.value


    def get_actions (self):
        actions = []

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                actions.append(Action("quit"))

            if event.type == pygame.KEYDOWN:
                if event.key in self.keydown:
                    actions.append(self.keydown[event.key])

            if event.type == pygame.KEYUP:
                if event.key in self.keyup:
                    actions.append(self.keyup[event.key])

            if event.type == pygame.JOYAXISMOTION:
                if event.axis in self.joy_axis:
                    actions.extend(
                        self.joy_axis[event.axis].
                            handle_value(event.value))

        return actions

    def puts (self, msg):
        print(msg)

    def update (self):
        pygame.display.update()

