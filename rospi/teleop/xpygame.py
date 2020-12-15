import pygame

# This implements the pygame interface. This can handle key-up events,
# so the motors stop as soon as you release the key. It create a tiny
# window just to accept events.
class PygameInterface:
    keydown = {
        pygame.K_ESCAPE:        "quit",
        pygame.K_SPACE:         "stop",
        pygame.K_w:             "forward",
        pygame.K_s:             "backward",
        pygame.K_a:             "left",
        pygame.K_d:             "right",
        pygame.K_UP:            "faster",
        pygame.K_DOWN:          "slower",
        pygame.K_RIGHT:         "turn-faster",
        pygame.K_LEFT:          "turn-slower",
    }

    keyup = {
        pygame.K_w:     "stop",
        pygame.K_s:     "stop",
        pygame.K_a:     "stop",
        pygame.K_d:     "stop",
    }

    def __init__ (self):
        self.win    = None
        self.joy    = None

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

    def get_actions (self):
        actions = []

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                actions.append("quit")

            if event.type == pygame.KEYDOWN:
                if event.key in self.keydown:
                    actions.append(self.keydown[event.key])

            if event.type == pygame.KEYUP:
                if event.key in self.keyup:
                    actions.append(self.keyup[event.key])

            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 0:
                    if event.value > 0.1:
                        actions.append("right")
                    elif event.value < -0.1:
                        actions.append("left")
                elif event.axis == 1:
                    if event.value > 0.1:
                        actions.append("backward")
                    elif event.value < -0.1:
                        actions.append("forward")

        return actions

    def puts (self, msg):
        print(msg)

    def update (self):
        pygame.display.update()

