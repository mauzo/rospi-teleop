from    __future__      import division
import  pygame
from    rospi.teleop    import Action, ControlAction

# Grr python2 rubbish
__metaclass__ = type

DEADZONE    = 0.05

class Axis:
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

# Colours
BLACK   = (0,   0,   0)
WHITE   = (255, 255, 255)
GRAY    = (80,  80,  80)
RED     = (240, 0,   0)
YELLOW  = (240, 240, 0)

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
        0:      Axis(action="turn", scale=1.0),
        1:      Axis(action="speed", scale=-1.0),
    }

    mouse_axis = (
        Axis(action="turn", scale=1.0),
        Axis(action="speed", scale=-1.0),
    )

    winsize = (400, 300)

    def __init__ (self):
        self.win        = None
        self.joy        = None
        self.mouse      = False

    def setup (self):
        self.setup_display()
        self.setup_joystick()

    def setup_display (self):
        pygame.display.init()
        self.win = pygame.display.set_mode(self.winsize)
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

    def handle_mouse_click (self):
        self.mouse = not self.mouse
        pygame.event.set_grab(self.mouse)
        pygame.mouse.set_visible(not self.mouse)

        if self.mouse:
            ws  = self.winsize
            pygame.mouse.set_pos(ws[0]/2, ws[1]/2)

    def handle_mouse_motion (self, pos):
        ws  = self.winsize
        rv  = []

        for i in 0, 1:
            sc  = ws[i]/2
            v   = (pos[i] - sc)/sc
            a   = self.mouse_axis[i].handle_value(v)
            rv.extend(a)

        return rv

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

            if event.type == pygame.MOUSEBUTTONDOWN:
                self.handle_mouse_click()

            if event.type == pygame.MOUSEMOTION:
                if self.mouse:
                    actions.extend(
                        self.handle_mouse_motion(event.pos))

        return actions

    def puts (self, msg):
        print(msg)

    def draw (self, state):
        w   = self.win
        d   = pygame.draw
        sp  = state.speed.scale(100)
        sps = state.speed.scale_step(100)
        tr  = state.turn.scale(100)
        trs = state.turn.scale_step(100)

        w.fill(BLACK)

        d.rect(w, GRAY, [150-trs, 125, 2*trs, 50], 0)
        if tr > 0:
            d.rect(w, RED, [150, 125, tr, 50], 0)
        elif tr < 0:
            d.rect(w, RED, [150+tr, 125, -tr, 50], 0)
        d.rect(w, WHITE, [50, 125, 200, 50], 2)
        d.line(w, WHITE, [150, 125], [150, 175], 2)

        d.rect(w, GRAY, [300, 150-sps, 50, 2*sps], 0)
        if sp > 0:
            d.rect(w, RED, [300, 150-sp, 50, sp], 0)
        elif sp < 0:
            d.rect(w, RED, [300, 150, 50, -sp], 0)
        d.rect(w, WHITE, [300, 50, 50, 200], 2)
        d.line(w, WHITE, [300, 150], [350, 150], 2)


    def update (self):
        pygame.display.update()

