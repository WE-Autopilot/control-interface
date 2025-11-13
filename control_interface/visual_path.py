import math 

from textual_canvas import Canvas 
from textual.color import Color

WHITE = Color.parse("white")
DIM = Color.parse("darkgrey")
RED = Color.parse("red")

# EXAMPLE WAYPOINTS
WAYPOINTS = [(0,0), (5,15), (10,20), (20,25)]

class PathCanvas(Canvas):
    refresh_rate = 10 # Hz

    def __init__(self, **kwargs):
        # width and height is PIXELS not SIZE!
        super().__init__(width=60, height=60, **kwargs)


    def on_mount(self):
        # update at 10hz
        self.set_interval(1 / self.refresh_rate, self.draw_path)

    def draw_path(self):
        width, height = self.width, self.height 

        # (0,0) is bottom center
        cx = width // 2
        cy = height - 1

        self.clear()

        # draw axes 
        for x in range(width):
            self.set_pixel(x, cy, DIM)
        for y in range(height):
            self.set_pixel(cx, y, DIM)
        
        # plot waypoints & connect with lines
        for i, (x, y) in enumerate(WAYPOINTS):
            sx = cx + x # shift origin to center
            sy = cy - y # flip y upward

            # connect to next waypoint
            if i < len(WAYPOINTS) - 1:
                nx, ny = WAYPOINTS[i + 1]
                nx, ny = cx + nx, cy - ny
                self.draw_line(sx, sy, nx, ny, WHITE)

        # draw waypoints on top
        for i, (x, y) in enumerate(WAYPOINTS):
            sx = cx + x # shift origin to center
            sy = cy - y # flip y upward
            self.set_pixel(sx, sy, RED)

 