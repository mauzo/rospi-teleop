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

    def setup (self):
        pygame.display.init()
        win = pygame.display.set_mode((100, 100))
        pygame.key.set_repeat(0)

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

        return actions

    def puts (self, msg):
        print(msg)

    def update (self):
        pygame.display.update()

