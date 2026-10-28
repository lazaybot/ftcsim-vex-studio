"""Shared visual language for Robot Simulation Studio."""
BG = (13, 17, 23)
PANEL = (18, 24, 32, 224)
PANEL_SOLID = (18, 24, 32)
BORDER = (51, 65, 81)
TEXT = (230, 235, 242)
MUTED = (139, 151, 166)
CYAN = (67, 196, 255)
ORANGE = (255, 171, 64)
GREEN = (82, 211, 137)
RED = (255, 100, 108)
FIELD = (31, 45, 58)


def panel(pygame, surface, rect, colour=PANEL, radius=8, border=True):
    layer = pygame.Surface(rect.size, pygame.SRCALPHA)
    pygame.draw.rect(layer, colour, layer.get_rect(), border_radius=radius)
    surface.blit(layer, rect.topleft)
    if border:
        pygame.draw.rect(surface, BORDER, rect, 1, border_radius=radius)
