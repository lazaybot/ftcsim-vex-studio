"""Simple autonomous path editor: click to add, drag to move, JSON save/load."""
import argparse
import math

import pygame

from autonomous import Path, Waypoint
import theme

SIZE = (1120, 720)
FIELD_METERS = 3.66
MARGIN = 55


def world_to_screen(x, y):
    scale = (SIZE[1] - 2 * MARGIN) / FIELD_METERS
    return int(SIZE[0] / 2 + x * scale), int(SIZE[1] / 2 - y * scale)


def screen_to_world(x, y):
    scale = (SIZE[1] - 2 * MARGIN) / FIELD_METERS
    return (x - SIZE[0] / 2) / scale, (SIZE[1] / 2 - y) / scale


def run(filename):
    pygame.init()
    screen = pygame.display.set_mode(SIZE)
    pygame.display.set_caption("ftcsim autonomous path editor")
    font = pygame.font.SysFont("monospace", 17)
    path = Path.load(filename) if filename else Path(name="autonomous")
    selected = None
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                path.save(filename or "autonomous_path.json")
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                if selected is not None:
                    path.waypoints.pop(selected); selected = None
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                hit = [i for i, w in enumerate(path.waypoints)
                       if math.dist(event.pos, world_to_screen(w.x, w.y)) < 14]
                if hit:
                    selected = hit[-1]
                else:
                    x, y = screen_to_world(*event.pos)
                    path.waypoints.append(Waypoint(round(x, 3), round(y, 3)))
                    selected = len(path.waypoints) - 1
            elif event.type == pygame.MOUSEMOTION and event.buttons[0] and selected is not None:
                x, y = screen_to_world(*event.pos)
                path.waypoints[selected].x, path.waypoints[selected].y = round(x, 3), round(y, 3)

        screen.fill(theme.BG)
        tl = world_to_screen(-FIELD_METERS / 2, FIELD_METERS / 2)
        br = world_to_screen(FIELD_METERS / 2, -FIELD_METERS / 2)
        pygame.draw.rect(screen, theme.FIELD, (*tl, br[0]-tl[0], br[1]-tl[1]), 0)
        pygame.draw.rect(screen, theme.BORDER, (*tl, br[0]-tl[0], br[1]-tl[1]), 2)
        points = [world_to_screen(w.x, w.y) for w in path.waypoints]
        if len(points) > 1: pygame.draw.lines(screen, theme.CYAN, False, points, 3)
        for i, point in enumerate(points):
            pygame.draw.circle(screen, theme.ORANGE if i == selected else theme.TEXT, point, 9)
            screen.blit(font.render(str(i + 1), True, theme.TEXT), (point[0] + 12, point[1] - 10))
        theme.panel(pygame, screen, pygame.Rect(14, 10, SIZE[0]-28, 38), colour=theme.PANEL)
        screen.blit(font.render("AUTONOMOUS PATH STUDIO", True, theme.TEXT), (26, 20))
        hint = "CLICK ADD/SELECT  •  DRAG MOVE  •  DELETE REMOVE  •  S SAVE"
        hs = font.render(hint, True, theme.MUTED)
        screen.blit(hs, (SIZE[0] - hs.get_width() - 26, 20))
        pygame.display.flip()
    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path", nargs="?", help="existing or output JSON path")
    run(parser.parse_args().path)
