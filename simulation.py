import pygame
import random
import math

scale = 1

# Screen dimensions
width, height = 800, 600

# Boid properties
num_boids = 10
max_speed = 4 * scale
max_force = 0.1 * scale
perception_radius = 50

# Obstacle properties
max_force_avoidance = 0.3 * scale
max_force_edges = 0.3 * scale

# Colors
white = (255, 255, 255)
red = (255, 0, 0)
black = (0, 0, 0)

class Boid:
    def __init__(self):
        self.position = pygame.math.Vector2(random.uniform(25, 75), random.uniform(25, 75))
        self.velocity = pygame.math.Vector2(random.uniform(-1, 1), random.uniform(-1, 1))
        self.velocity.scale_to_length(max_speed)
        self.acceleration = pygame.math.Vector2(0, 0)

    def apply_behavior(self, boids, obstacles, walls):
        self.acceleration = pygame.math.Vector2(0, 0)
        self.acceleration += self.avoid_edges()
        self.acceleration += self.align(boids)
        self.acceleration += self.cohesion(boids)
        self.acceleration += self.separation(boids)
        self.acceleration += self.avoid_obstacle(obstacles)
        self.acceleration += self.avoid_walls(walls)
        
    def update(self):
        self.velocity += self.acceleration
        if self.velocity.length() > max_speed:
            self.velocity.scale_to_length(max_speed)
        self.position += self.velocity

    def show(self, screen):
        angle = self.velocity.angle_to(pygame.math.Vector2(1, 0))
        pygame.draw.polygon(screen, white, [
            self.position + pygame.math.Vector2(math.cos(math.radians(angle)), -math.sin(math.radians(angle))) * 10,
            self.position + pygame.math.Vector2(math.cos(math.radians(angle + 160)), -math.sin(math.radians(angle + 160))) * 10,
            self.position + pygame.math.Vector2(math.cos(math.radians(angle - 160)), -math.sin(math.radians(angle - 160))) * 10,
        ])

    def align(self, boids):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        for boid in boids:
            if boid != self and self.position.distance_to(boid.position) < perception_radius:
                steering += boid.velocity
                total += 1
        if total > 0:
            steering /= total
            steering.scale_to_length(max_speed)
            steering -= self.velocity
            if steering.length() > max_force:
                steering.scale_to_length(max_force)
        return steering

    def cohesion(self, boids):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        for boid in boids:
            if boid != self and self.position.distance_to(boid.position) < perception_radius:
                steering += boid.position
                total += 1
        if total > 0:
            steering /= total
            steering -= self.position
            steering.scale_to_length(max_speed)
            steering -= self.velocity
            if steering.length() > max_force:
                steering.scale_to_length(max_force)
        return steering

    def separation(self, boids):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        for boid in boids:
            distance = self.position.distance_to(boid.position)
            if boid != self and distance < perception_radius:
                diff = self.position - boid.position
                diff /= distance**2
                steering += diff
                total += 1
        if total > 0:
            steering /= total
            steering.scale_to_length(max_speed)
            steering -= self.velocity
            if steering.length() > max_force:
                steering.scale_to_length(max_force)
        return steering
    
    def show_perception(self, boids, screen):
        for boid in boids:
            if boid != self and self.position.distance_to(boid.position) < perception_radius:
                pygame.draw.line(screen, red, self.position, boid.position, 2)
    
    def avoid_edges(self):
        steering = pygame.math.Vector2(0, 0)
        buffer = perception_radius  # Distance from edge to start avoiding
        if self.position.x < buffer:
            steering += pygame.math.Vector2(max_speed, 0)
        elif self.position.x > width - buffer:
            steering += pygame.math.Vector2(-max_speed, 0)
        if self.position.y < buffer:
            steering += pygame.math.Vector2(0, max_speed)
        elif self.position.y > height - buffer:
            steering += pygame.math.Vector2(0, -max_speed)
        if steering.length() > 0:
            steering.scale_to_length(max_speed)
            steering -= self.velocity
            if steering.length() > max_force_edges:
                steering.scale_to_length(max_force_edges)
        return steering

    def avoid_obstacle(self, obstacles):
        steering = pygame.math.Vector2(0, 0)
        for obstacle in obstacles:
            distance = self.position.distance_to(obstacle.position)
            if distance < obstacle.radius + perception_radius:
                diff = self.position - obstacle.position
                diff /= distance
                steering += diff
        if steering.length() > 0:
            steering.scale_to_length(max_speed)
            steering -= self.velocity
            if steering.length() > max_force_avoidance:
                steering.scale_to_length(max_force_avoidance)
        return steering
    
    def avoid_walls(self, walls):
        steering = pygame.math.Vector2(0, 0)
        for wall in walls:
            distance = wall.distance_to(self.position)
            if distance < perception_radius:
                # Get vector perpendicular to the wall
                perp = wall.get_perpendicular(self.position)
                steering += perp
        if steering.length() > 0:
            steering.scale_to_length(max_speed)
            steering -= self.velocity
            if steering.length() > max_force_avoidance:
                steering.scale_to_length(max_force_avoidance)
        return steering

class Obstacle:
    def __init__(self, x, y, radius):
        self.position = pygame.math.Vector2(x, y)
        self.radius = radius
    
    def show(self, screen):
        pygame.draw.circle(screen, red, (int(self.position.x), int(self.position.y)), self.radius)

class Wall:
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start = pygame.math.Vector2(start_x, start_y)
        self.end = pygame.math.Vector2(end_x, end_y)
    
    def show(self, screen):
        pygame.draw.line(screen, red, self.start, self.end, 5)
    
    def distance_to(self, point):
        # find the closest distance
        line_vector = self.end - self.start
        point_vector = point - self.start
        line_length = line_vector.length()

        # Project the point onto the line
        t = max(0, min(1, point_vector.dot(line_vector) / (line_length ** 2)))
        projection = self.start + t * line_vector

        # Return the distance from the point to the projection
        return point.distance_to(projection)
    
    def get_perpendicular(self, point):
        # for force away
        line_vector = self.end - self.start
        perp_vector = pygame.math.Vector2(-line_vector.y, line_vector.x)
        if (point - self.start).dot(perp_vector) < 0:
            perp_vector = -perp_vector
        return perp_vector.normalize()

def main():
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()

    boids = [Boid() for _ in range(num_boids)]

    # Create obstacles
    obstacles = [
        # Obstacle(200, 300, 40),
        # Obstacle(400, 200, 50),
        # Obstacle(600, 400, 30),
    ]

    # Create walls
    walls = [
        Wall(100, 000, 100, 500),
        Wall(300, 100, 300, 600),
        Wall(500, 000, 500, 500),
        Wall(700, 100, 700, 600),
    ]


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(black)

        for boid in boids:
            boid.show_perception(boids, screen)         # show nearby boids
            boid.apply_behavior(boids, obstacles, walls)       # incl. avoid obstacles and edges
            boid.update()
            boid.show(screen)
        
        for obstacle in obstacles:
            obstacle.show(screen)

        for wall in walls:
            wall.show(screen)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    main()
