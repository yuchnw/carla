import pygame
import numpy as np
import argparse
from sensor_msgs import point_cloud2
import rosbag

# --- Parse command-line arguments ---
parser = argparse.ArgumentParser(description="Visualize LiDAR data from ROS bag using Pygame")
parser.add_argument("bagfile", help="Path to the ROS bag file")
args = parser.parse_args()

# --- Pygame setup ---
WIDTH, HEIGHT = 1600, 800
SCALE = 0.1  # meters per pixel
CENTER = (WIDTH // 2, HEIGHT // 2)
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("ROS LiDAR Viewer")

# --- Open ROS bag ---
bag = rosbag.Bag(args.bagfile)
topic = '/aeva_left/points'

# --- Main loop ---
running = True
clock = pygame.time.Clock()

for _, msg, _ in bag.read_messages(topics=[topic]):
    # Convert PointCloud2 to XYZ
    points = list(point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))

    # Draw in Pygame
    screen.fill((0, 0, 0))
    for x, y, _ in points:
        px = int(CENTER[0] + x / SCALE)
        py = int(CENTER[1] - y / SCALE)
        if 0 <= px < WIDTH and 0 <= py < HEIGHT:
            pygame.draw.circle(screen, (0, 255, 0), (px, py), 1)

    pygame.display.flip()
    clock.tick(10)  # Adjust frame rate

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
    if not running:
        break

bag.close()
pygame.quit()