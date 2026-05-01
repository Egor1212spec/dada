import struct

width, height = 1024, 1024
img = [[(255, 255, 255) for _ in range(width)] for _ in range(height)]

def draw_thick_line(x0, y0, x1, y1, thickness):
    min_x = int(max(0, min(x0, x1) - thickness))
    max_x = int(min(width-1, max(x0, x1) + thickness))
    min_y = int(max(0, min(y0, y1) - thickness))
    max_y = int(min(height-1, max(y0, y1) + thickness))
    
    dx = x1 - x0
    dy = y1 - y0
    length_sq = dx*dx + dy*dy
    if length_sq == 0: return
        
    for y in range(min_y, max_y + 1):
        for x in range(min_x, max_x + 1):
            t = max(0, min(1, ((x - x0) * dx + (y - y0) * dy) / length_sq))
            proj_x = x0 + t * dx
            proj_y = y0 + t * dy
            dist_sq = (x - proj_x)**2 + (y - proj_y)**2
            if dist_sq <= (thickness/2)**2:
                img[y][x] = (0, 0, 0)

# Лабиринт с перекрестками для Left-Hand Rule
lines = [
    ((150, 150), (150, 900)), # Main vertical left
    ((150, 400), (350, 400)), # Right branch
    ((50, 600), (150, 600)),  # Left branch
    ((150, 800), (450, 800)), # Right branch (Correct path)
    ((450, 900), (450, 200)), # Vertical at 450
    ((250, 500), (450, 500)), # Left branch from 450
    ((450, 600), (650, 600)), # Right branch from 450
    ((450, 200), (750, 200)), # Turn at 450, 200 to right
    ((750, 100), (750, 800)), # Up at 750
]

for line in lines:
    draw_thick_line(line[0][0], line[0][1], line[1][0], line[1][1], 1)

# Финишная зона (большой черный квадрат)
for y in range(790, 810):
    for x in range(740, 760):
        if 0 <= y < height and 0 <= x < width:
            img[y][x] = (0, 0, 0)

with open('track.bmp', 'wb') as f:
    filesize = 54 + 3 * width * height
    f.write(b'BM')
    f.write(struct.pack('<L', filesize))
    f.write(b'\x00\x00\x00\x00')
    f.write(struct.pack('<L', 54))
    
    f.write(struct.pack('<L', 40))
    f.write(struct.pack('<L', width))
    f.write(struct.pack('<L', height))
    f.write(struct.pack('<H', 1))
    f.write(struct.pack('<H', 24))
    f.write(struct.pack('<L', 0))
    f.write(struct.pack('<L', 3 * width * height))
    f.write(struct.pack('<L', 2835))
    f.write(struct.pack('<L', 2835))
    f.write(struct.pack('<L', 0))
    f.write(struct.pack('<L', 0))
    
    for y in range(height-1, -1, -1):
        for x in range(width):
            r, g, b = img[y][x]
            f.write(struct.pack('BBB', b, g, r))

print("Maze track generated successfully.")
