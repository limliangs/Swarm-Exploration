import pygame
import random
import math
import concurrent.futures

dt = 1

# Screen dimensions
width, height = 1200, 700

# Boid properties
num_boids = 10          # adjust number of boids
max_rank = 7            # default rank
min_step = 1.5          # set minimum movement for swarming
max_speed = 4           # physical speed limit
max_force = 0.15        # maximum acceleration due to al, coh, sep
perception_radius = 200
safe_distance = 150     # distance which separation starts to be applied

# Weights for each behaviour
weight_al = 0.2
weight_coh = 0.2
weight_sep = 0.4
weight_edge = 0.4
weight_targ = 3

# Point of Interest properties
POI_radius = 20

# Colors
white = (255, 255, 255)
red = (255, 0, 0)
black = (0, 0, 0)
green = (0, 255, 0)
pink = (255, 100, 100)

class Boid:
    def __init__(self, label):
        self.position = pygame.math.Vector2(random.uniform(0, width), random.uniform(0, height))    # randomize starting position
        self.velocity = pygame.math.Vector2(random.uniform(-1, 1), random.uniform(-1, 1))           # randomize velocity, scale to max_speed
        self.velocity.scale_to_length(max_speed)
        self.acceleration = pygame.math.Vector2(0, 0)                                               # initialize acceleration = 0
        self.angle = self.velocity.angle_to(pygame.math.Vector2(1, 0))                              # angle for visual aid when shown
        self.mode = 0                                                                               # 0 = swarming, 1 = approaching, 2 = arrive
        self.targets = []
        self.ranks = []
        self.removed_targets = []
        self.label = label

    def apply_behavior(self, boids, POIs):          # Executed in "parallel" with other boids
        neighbours = self.get_neighbours(boids)

        # Acceleration by al, col, sep, edges
        n_acceleration = pygame.math.Vector2(0, 0)  
        al = self.cap(self.align(neighbours))
        coh = self.cap(self.cohesion(neighbours))
        sep = self.cap(self.separation(neighbours))
        edge = self.cap(self.avoid_edges())
        n_acceleration = weight_al * al + weight_coh * coh + weight_sep * sep + weight_edge * edge
        
        # Acceleration by follow_target
        selected = self.select_target()
        if selected != None:
            self.mode = 1
            targ = self.cap(self.follow_target(selected))
            n_acceleration += weight_targ * targ
            n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge + weight_targ)
        else:
            n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge)

        # Apply swarming behaviour (min_s)
        if self.mode == 0:
            n_acceleration = self.bound_to_s_min(n_acceleration)

        # Rank and targets
        new_targets, new_ranks, new_removed_targets = self.update_POIs(POIs, neighbours)

        targets = []
        for target in self.targets:
            targets.append(target.label)
        new_targetss = []
        for target in new_targets:
            new_targetss.append(target.label)

        print(f"{self.label}: targets: {targets} -> {new_targetss}, rank: {self.ranks} -> {new_ranks}")
        return n_acceleration, new_targets, new_ranks, new_removed_targets                       # returns calculated rank, and acceleration for next time step    

    def update_POIs(self, POIs, neighbours):
        new_targets = self.targets
        new_ranks = self.ranks
        # Scan neighbour's POIs, rank accordingly
        for neighbour in neighbours:
            for target in neighbour.targets:
                if target not in new_targets:
                    new_targets.append(target)
                    new_ranks.append(self.rank_neighbours(neighbours, target))
        # Scan radius, claim leader
        for poi in POIs:
            if self.position.distance_to(poi.position) < perception_radius:
                if poi not in new_targets:
                    new_targets.append(poi)
                    new_ranks.append(0)
        
        new_removed_targets = []
        # Check target within self radius to remove
        for target in self.targets:
            count = 0
            if self.position.distance_to(target.position) <= POI_radius:
                count += 1
                for neighbour in neighbours:
                    if neighbour.position.distance_to(target.position) <= POI_radius:
                        count += 1
                        if count == 3:
                            index = new_targets.index(target)
                            new_targets.remove(target)
                            new_removed_targets.append(target)   # remove from new_targets, append to removed_targets
                            new_ranks.pop(index)
        # Check target within neighbours removed targets
        for neighbour in neighbours:
            for target in neighbour.removed_targets:
                if target in new_targets:
                    index = new_targets.index(target)
                    new_targets.remove(target)
                    new_removed_targets.append(target)
                    new_ranks.pop(index)
        return new_targets, new_ranks, new_removed_targets
    
    def rank_neighbours(self, neighbours, target):
        min_rank = max_rank
        for neighbour in neighbours:
            try:                                            # If target is in neighbour.targets
                index = neighbour.targets.index(target)     # Index of targets and ranks correspond to each other (hopefully)
                if neighbour.ranks[index] < min_rank:
                    min_rank = neighbour.ranks[index]
            except ValueError:
                pass
        return min_rank

    def cap(self, acc):
        centre = -self.velocity/dt
        if acc.length() > 0:
            if acc.length() > max_force:
                scaled_acc = acc / acc.length() * max_force
                if scaled_acc.distance_to(centre) < max_speed/dt:
                    steering = scaled_acc
                else:
                    acc_from_centre = acc - centre
                    steering = acc_from_centre / acc_from_centre.length() * max_speed/dt + centre
            else:
                if acc.distance_to(centre) <= max_speed/dt:
                    steering = acc
                else:
                    acc_from_centre = acc - centre
                    steering = acc_from_centre / acc_from_centre.length() * max_speed/dt + centre
        else:
            steering = pygame.math.Vector2(0,0)
        return steering

    def bound_to_s_min(self, acc):                  # only do when mode == 0
        # randomize velocity once it finishes
        centre = -2 * self.velocity / dt
        if acc.distance_to(centre) >= 2 * min_step / dt**2:
            n_acceleration = acc
        else:
            acc_from_centre = acc - centre
            n_acceleration = (acc_from_centre / acc_from_centre.length() * 2 * min_step /dt**2) + centre
        return n_acceleration

    def update(self, n_acceleration, new_targets, new_ranks, new_removed_targets):
        # update attributes from the previously computed values
        self.acceleration = self.cap(n_acceleration)
        self.targets = new_targets
        self.ranks = new_ranks
        self.removed_targets = new_removed_targets
        # freeze if arrived
        if self.mode == 2:      
            self.freeze()
        # update angle, and bound & apply kinematics
        else:
            self.get_angle()
            self.position += self.velocity * dt + 0.5 * self.acceleration * dt**2
            self.velocity += self.acceleration * dt

    def show_perception(self, neighbours, screen):  # Draws lines to neighbours
        for neighbour in neighbours:
            if self.position.distance_to(neighbour.position) < perception_radius:
                pygame.draw.line(screen, red, self.position, neighbour.position, 2)

    def freeze(self):
        # freeze when arrived
        self.velocity = pygame.math.Vector2(0, 0)
        self.acceleration = pygame.math.Vector2(0, 0)

    def get_angle(self): 
        # get angle before stop
        if self.velocity.length != 0:
                self.angle = self.velocity.angle_to(pygame.math.Vector2(1, 0))

    def show(self, screen, font):
        pygame.draw.polygon(screen, white, [
            self.position + pygame.math.Vector2(math.cos(math.radians(self.angle)), -math.sin(math.radians(self.angle))) * 10,
            self.position + pygame.math.Vector2(math.cos(math.radians(self.angle + 160)), -math.sin(math.radians(self.angle + 160))) * 10,
            self.position + pygame.math.Vector2(math.cos(math.radians(self.angle - 160)), -math.sin(math.radians(self.angle - 160))) * 10,
        ])

        # Print rank on screen
        info_text = font.render(f"{self.label} ", True, white)
        screen.blit(info_text, (self.position.x + 15, self.position.y - 10))

    def select_target(self):
        selected = None
        self.mode = 0
        closest = perception_radius + 1
        for target in self.targets:
            distance = self.position.distance_to(target.position)
            if distance <= perception_radius:
                if distance < closest:
                    closest = distance
                    selected = target
        return selected

    def follow_target(self, target):
        steering = pygame.math.Vector2(0, 0)
        if target != None:
            distance = self.position.distance_to(target.position)
            if distance <= POI_radius:
                self.mode = 2                                           # freeze
            steering = target.position - self.position                  # Delta p
            steering = 2 * (steering - self.velocity * dt) / dt**2      # acceleration required to achieve Deltap
        return steering

    def get_neighbours(self, boids):
        neighbours = []
        for boid in boids:
            if boid != self and self.position.distance_to(boid.position) < perception_radius:
                neighbours.append(boid)
        return neighbours                   # returns a list of boids that are within perception_radius

    def align(self, neighbours):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        for neighbour in neighbours:
            steering += neighbour.velocity                          # total velocity
            total += 1
        if total > 0:
            steering = steering/total                                   # avg velocity
            steering = (steering - self.velocity) / dt                  # acceleration needed to match avg velocity at next dt
        return steering

    def cohesion(self, neighbours):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        for neighbour in neighbours:
            steering += neighbour.position                              # total position
            total += 1
        if total > 0:
            steering /= total                                           # avg position
            steering -= self.position                                   # Delta p
            steering = 2 * (steering - self.velocity * dt) / dt**2      # acceleration needed to arrive at centre of mass at next dt
        return steering

    def separation(self, neighbours):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        for neighbour in neighbours:
            distance = self.position.distance_to(neighbour.position)
            if distance < safe_distance:
                away_v = -(neighbour.position - self.position)          # vector pointing away
                away_v.scale_to_length(safe_distance)                   # scaled vector to match safe_dist
                goal_p = neighbour.position + away_v                    # goal position w.r.t. one particular neighbour
                steering += goal_p                                      
                total += 1
        if total > 0:
            steering /= total                                           # avg goal position
            steering -= self.position                                   # Delta p
            steering = 2 * (steering - self.velocity * dt) / dt**2      # acceleration needed to arrive at centre of mass at next dt
        return steering

    def avoid_edges(self):
        steering = pygame.math.Vector2(0, 0)
        buffer = perception_radius  # Distance from edge to start avoiding
        
        if self.position.x < buffer:
            steering += pygame.math.Vector2(max_speed, 0)
            if self.position.x <= 0:
                self.velocity.x *= -1  # Bounce
        elif self.position.x > width - buffer:
            steering += pygame.math.Vector2(-max_speed, 0)
            if self.position.x >= width:
                self.velocity.x *= -1  # Bounce

        if self.position.y < buffer:
            steering += pygame.math.Vector2(0, max_speed)
            if self.position.y <= 0:
                self.velocity.y *= -1  # Bounce
        elif self.position.y > height - buffer:
            steering += pygame.math.Vector2(0, -max_speed)
            if self.position.y >= height:
                self.velocity.y *= -1  # Bounce

        if steering.length() > 0:
            steering.scale_to_length(max_speed)
            steering -= self.velocity
        return steering
    
    def wraparound(self):
        if self.position.x < 0:
            self.position.x = (width + (self.position.x % width))
        if self.position.x >= width:
            self.position.x %= width
        if self.position.y < 0:
            self.position.y = (height + (self.position.y % height))
        if self.position.y >= height:
            self.position.y %= height

    def bounce(self):
        if self.position.x <= 0 or self.position.x >= width:
            self.velocity.x *= -1  # Bounce
        if self.position.y <= 0 or self.position.y >= height:
            self.velocity.x *= -1  # Bounce

class POI:
    def __init__(self, x, y, label):
        self.position = pygame.math.Vector2(x,y)
        self.count = 0
        self.label = label
    
    def show(self, screen, font):
        pygame.draw.circle(screen, pink, (self.position.x, self.position.y), POI_radius)
        pygame.draw.circle(screen, red, (self.position.x, self.position.y), 10)
        # Print rank on screen
        info_text = font.render(f"{self.label}", True, white)
        screen.blit(info_text, (self.position.x + 15, self.position.y - 10))

    def update(self, boids, POIs, screen):  # remove self if task is completed
        self.count = 0
        for boid in boids:
            distance = self.position.distance_to(boid.position)
            if distance < perception_radius:
                pygame.draw.line(screen, pink, self.position, boid.position, 2)
                if distance < POI_radius:
                    self.count += 1
                    if self.count == 3:
                        POIs.remove(self)

def main():
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()

    boids = [Boid(chr(65 + i)) for i in range(num_boids)]
    POIs = []

    ts = 0  # Time step
    i = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(black)

        mouse = pygame.mouse.get_pressed()
        # Create POI at mouse click
        if mouse[0] == 1:
            x, y = pygame.mouse.get_pos()
            POIs.append(POI(x, y, chr(97 + i)))
            i += 1
        
        # Draw POI
        for poi in POIs:
            poi.update(boids, POIs, screen)
            poi.show(screen, font=pygame.font.Font(None, 24))

        # Use ThreadPoolExecutor to update each boid in parallel
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Apply behaviors and update boids concurrently
            futures = [executor.submit(boid.apply_behavior, boids, POIs) for boid in boids]
            concurrent.futures.wait(futures)  # Wait for all boids to complete calculation
            
            new_values = [future.result() for future in futures]
            for boid, (n_acceleration, new_targets, new_ranks, new_removed_targets) in zip(boids, new_values):
                boid.update(n_acceleration, new_targets, new_ranks, new_removed_targets)

        # Draw boids
        for boid in boids:
            boid.show_perception(boids, screen)
            boid.show(screen, font=pygame.font.Font(None, 24))

        print(f"Time Step: {ts}")
        ts += 1

        pygame.display.flip()
        # Adjust to see in slow motion. Try 15 or 10
        clock.tick(15)


    pygame.quit()

if __name__ == "__main__":
    main()