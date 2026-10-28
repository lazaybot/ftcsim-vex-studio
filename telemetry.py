"""Small dependency-free telemetry recorder and pygame graph renderer."""
from collections import deque
import csv
from pathlib import Path
import time
import theme


class Telemetry:
    def __init__(self, max_samples=900):
        self.started = time.perf_counter()
        self.samples = deque(maxlen=max_samples)

    def add(self, **values):
        self.samples.append({"time": time.perf_counter() - self.started, **values})

    def series(self, key):
        return [(row["time"], row[key]) for row in self.samples if key in row]

    def export_csv(self, path):
        path = Path(path)
        keys = sorted({k for row in self.samples for k in row if k != "time"})
        with path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["time", *keys])
            writer.writeheader()
            writer.writerows(self.samples)
        return path

    def draw(self, pygame, surface, rect, keys=("speed", "heading_error")):
        theme.panel(pygame, surface, rect)
        font = pygame.font.SysFont("monospace", 13)
        title = font.render("LIVE TELEMETRY", True, theme.TEXT)
        surface.blit(title, (rect.x + 10, rect.y + 8))
        palette = [theme.CYAN, theme.ORANGE, theme.GREEN]
        for index, key in enumerate(keys):
            values = [float(v) for _, v in self.series(key)]
            if len(values) < 2:
                continue
            limit = max(1e-6, max(abs(v) for v in values))
            points = []
            for i, value in enumerate(values):
                x = rect.x + 8 + i * (rect.w - 16) / max(1, len(values) - 1)
                y = rect.centery + 8 - value / limit * (rect.h * 0.28)
                points.append((x, y))
            pygame.draw.lines(surface, palette[index % len(palette)], False, points, 2)
            label = font.render(key.replace("_", " "), True, palette[index % len(palette)])
            surface.blit(label, (rect.x + 10 + index * 125, rect.bottom - 21))

