"""Live controller inspector with axes, buttons, dead-zone and event log."""
import pygame

from teleop import deadzone


def run():
    pygame.init(); pygame.joystick.init()
    screen = pygame.display.set_mode((900, 620))
    pygame.display.set_caption("ftcsim controller test")
    font = pygame.font.SysFont("monospace", 18)
    clock = pygame.time.Clock(); log = []
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
            elif event.type in (pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP, pygame.JOYAXISMOTION, pygame.JOYHATMOTION):
                log.append(str(event)); log = log[-10:]
        screen.fill((18, 22, 28))
        if not pygame.joystick.get_count():
            rows = ["No controller detected.", "Connect one, then restart this test."]
        else:
            js = pygame.joystick.Joystick(0); js.init()
            rows = [f"{js.get_name()}   axes={js.get_numaxes()} buttons={js.get_numbuttons()} hats={js.get_numhats()}"]
            rows += [f"axis {i:2}: raw {js.get_axis(i):+0.3f}   filtered {deadzone(js.get_axis(i)):+0.3f}" for i in range(js.get_numaxes())]
            rows += ["buttons: " + " ".join(f"{i}:{js.get_button(i)}" for i in range(js.get_numbuttons()))]
            rows += ["", "Recent events:", *log]
        for i, row in enumerate(rows): screen.blit(font.render(row, True, (225, 232, 240)), (24, 24 + i * 27))
        pygame.display.flip(); clock.tick(60)
    pygame.quit()


if __name__ == "__main__": run()
