import pygame
import random
import math
import concurrent.futures

dt = 1

# Screen dimensions
width, height = 1200, 700

# Boid properties
num_boids = 20          # adjust number of boids
max_rank = 7            # default rank
min_speed = 1           # set minimum movement for swarming
max_speed = 3           # physical speed limit
max_force = 0.5        # maximum acceleration due to al, coh, sep
perception_radius = 200
safe_distance = 150     # distance which separation starts to be applied

# Weights for each behaviour
weight_al = 0.2
weight_coh = 0.2
weight_sep = 0.4
weight_edge = 0
weight_target = 6

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
        self.velocity.scale_to_length(0)
        self.acceleration = pygame.math.Vector2(0, 0)                                               # initialize acceleration = 0
        self.angle = self.velocity.angle_to(pygame.math.Vector2(1, 0))                              # angle for visual aid when shown
        self.mode = 0                                                                               # 0 = swarming, 1 = approaching, 2 = arrive
        self.rank = max_rank                                                                        # initialize rank to 7
        self.connected = False
        self.label = label
        self.target = None

    def apply_behavior(self, boids, POIs):          # Executed in "parallel" with other boids
        neighbours = self.get_neighbours(boids)

        # Acceleration by al, col, sep, edges
        n_acceleration = pygame.math.Vector2(0, 0)  
        al = (self.align(neighbours))
        coh = (self.cohesion(neighbours))
        sep, weight_sep = self.separation(neighbours)
        sep = (sep)
        edge = (self.avoid_edges())
        n_acceleration = weight_al * al + weight_coh * coh + weight_sep * sep + weight_edge * edge
        
        # Acceleration by target, weighted average
        self.target = self.get_target(POIs)
        if self.target != None:
            self.mode = 1
            tar = (self.follow_target(self.target))
            n_acceleration += weight_target * tar
            n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge + weight_target)
        else:
            n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge)
        # Capping
        n_acceleration_cap, modi = self.capping(n_acceleration)

        # Rank
        leader_connected = self.check_leader(boids)         # returns True or False, checks if leader is in the network
        if leader_connected == True:
            n_rank = self.ranking(neighbours, self.target)       # rank self based on neighbours
        else:
            if self.target == None:
                n_rank = 7
            else:
                n_rank = 0
        
        # Target removal
        self.check_done(neighbours)
        
        # Debug message
        print(f"{self.label}: type: {modi}, acc: {n_acceleration}, cap_acc: {n_acceleration_cap}, vel: {self.velocity}")
        
        # Return next acceleration and rank
        return n_rank, n_acceleration_cap
   
    def check_cond(self, centre, acc):              # Check if all 3 conditions are fulfiled
        if acc.length() <= max_force and acc.distance_to(centre) <= max_speed/dt and acc.distance_to(centre) >= min_speed/dt:
            return True

    def capping(self, acc):
        centre = -self.velocity/dt
        # Check if original acc satisfies
        if self.check_cond(centre, acc) == True:
            modi = "ORIGNL"
            return acc, modi
        # Try scaling acc to max_force
        if acc.length() > max_force:                                                            # Only scale if its too long
            scaled_acc = acc / acc.length() * max_force
            if self.check_cond(centre, scaled_acc) == True:
                modi = "SCALED"
                return scaled_acc, modi
        # Try projecting acc to max_speed or min_speed
        if acc != centre:                                                                       # Prevent division by 0
            proj_maxv_acc = (acc - centre) / (acc - centre).length() * max_speed/dt + centre    # Projection to outer circle
            proj_minv_acc = (acc - centre) / (acc - centre).length() * min_speed/dt + centre    # Projection to inner circle
            if acc.distance_to(proj_maxv_acc) < acc.distance_to(proj_minv_acc):                 # Select closer projection
                proj_acc = proj_maxv_acc
            else:
                proj_acc = proj_minv_acc
            if self.check_cond(centre, proj_acc) == True:
                modi = "PROJCT"
                return proj_acc, modi
        # Try getting the vertices of feasible set, return closest to acc
        a, b = self.intersection_vertices(centre_a=pygame.math.Vector2(0,0), radius_a=max_force, centre_b=centre, radius_b=max_speed)
        c, d = self.intersection_vertices(centre_a=pygame.math.Vector2(0,0), radius_a=max_force, centre_b=centre, radius_b=min_speed)
        points = [a, b, c, d]
        closest_distance = float('inf')
        selected_acc = None
        for point in points:
            if point != None:
                distance = point.distance_to(acc)
                if distance < closest_distance:
                    closest_distance = distance
                    selected_acc = point
        if selected_acc != None:
            if self.check_cond(centre, selected_acc) == True:
                modi = "INTSCT"
                return selected_acc, modi
        # BELOW SHOULD NOT BE NEEDED
        # if 0 return 0
        if self.check_cond(centre, acc=pygame.math.Vector2(0,0)) == True:
            modi = "ZERO A"
            return pygame.math.Vector2(0,0), modi
        # if no case
        modi = "NOCASE"
        return pygame.math.Vector2(0,0), modi

    def intersection_vertices(self, centre_a, radius_a, centre_b, radius_b):
        d = centre_a.distance_to(centre_b)
        if d > radius_a + radius_b:             # Circles dont intersect
            return None, None
        elif d < abs(radius_a - radius_b):      # One circle is contained within the other
            return None, None
        elif d == 0 and radius_a == radius_b:   # The circles are identical and overlap completely.")
            return None, None
        
        # Point P where the line through the intersection points crosses the line between centers
        a = (radius_a**2 - radius_b**2 + d**2) / (2 * d)
        P = centre_a + a * (centre_b - centre_a).normalize()
        
        # Distance from point P to the intersection points
        h = math.sqrt(radius_a**2 - a**2)
        
        # Offset vector perpendicular to the line between centers
        offset = (centre_b - centre_a).normalize().rotate(90) * h
        
        # Intersection points in Vector2 format
        intersection_1 = P + offset
        intersection_2 = P - offset

        return intersection_1, intersection_2    

    def cap(self, acc):     # add min_v bound
        found = False
        centre = -self.velocity/dt
        if acc.length() > 0:
            if acc.length() > max_force:
                scaled_acc = acc / acc.length() * max_force
                if scaled_acc.distance_to(centre) <= max_speed/dt:
                    steering = scaled_acc
                    found = True
                else:
                    acc_from_centre = acc - centre
                    steering = acc_from_centre / acc_from_centre.length() * max_speed/dt + centre
                    if steering.length() <= max_force:
                        found = True
            else:
                if acc.distance_to(centre) <= max_speed/dt:
                    steering = acc
                    found = True
                else:
                    acc_from_centre = acc - centre
                    steering = acc_from_centre / acc_from_centre.length() * max_speed/dt + centre
                    if steering.length() <= max_force:
                        found = True
        else:
            steering = pygame.math.Vector2(0,0)
            return steering
        if found:
            return steering
        # find intersection 
        return steering
 
    def bound_to_s_min(self, acc):                  # only do when mode == 0
        # randomize velocity once it finishes
        centre = -2 * self.velocity / dt
        if acc.distance_to(centre) >= 2 * min_speed / dt**2:
            n_acceleration = acc
        else:
            acc_from_centre = acc - centre
            n_acceleration = (acc_from_centre / acc_from_centre.length() * 2 * min_speed /dt**2) + centre
        return n_acceleration

    def update(self, n_rank, n_acceleration):
        # update attributes from the previously computed values
        self.rank = n_rank
        self.acceleration = n_acceleration
        # self.acceleration = self.cap(n_acceleration)
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

    def check_done(self, neighbours):
        if self.mode == 2:
            count = 1
            for neighbour in neighbours:
                if (neighbour.position - self.target.position).length() <= POI_radius and neighbour.mode == 2:
                    count += 1
            if count >= 3:
                self.target = None

    def freeze(self):
        # freeze when arrived
        self.velocity = pygame.math.Vector2(0, 0)
        self.acceleration = pygame.math.Vector2(0, 0)

    def get_angle(self): 
        # get angle before stop
        if self.velocity.length != 0:
                self.angle = self.velocity.angle_to(pygame.math.Vector2(1, 0))

    def check_leader(self, boids):                  # Search if leader is in network
        visited = set()                                 # Use a set to avoid duplicates

        # Recursive DFS function
        def dfs(boid):
            if boid.rank == 0:                          # If the boid is a leader
                return True
            visited.add(boid)                           # Mark the current boid as visited
            neighbours = boid.get_neighbours(boids)     # Get the neighbours of the current boid
            for neighbour in neighbours:
                if neighbour not in visited:            # Only visit unvisited neighbours
                    if dfs(neighbour):                  # Recur for the neighbours
                        return True
            return False

        # Start the DFS from the current boid
        return dfs(self)

    def ranking(self, neighbours, target):
        n_rank = self.rank
        if self.rank == 0:
            if target == None:
                n_rank = 7                          # if self was leader, and now target is gone, set next rank to 7
        else:
            min_rank = 7
            for neighbour in neighbours:
                if neighbour.rank < min_rank:
                    min_rank = neighbour.rank
            n_rank = min_rank + 1     # rank self based on neighbours' lowest rank + 1
        return n_rank

    def show(self, screen, font):
        # Draw leader as green
        if self.rank == 0:
            pygame.draw.polygon(screen, green, [
                self.position + pygame.math.Vector2(math.cos(math.radians(self.angle)), -math.sin(math.radians(self.angle))) * 10,
                self.position + pygame.math.Vector2(math.cos(math.radians(self.angle + 160)), -math.sin(math.radians(self.angle + 160))) * 10,
                self.position + pygame.math.Vector2(math.cos(math.radians(self.angle - 160)), -math.sin(math.radians(self.angle - 160))) * 10,
            ])
        # Draw followers as white
        else:
            pygame.draw.polygon(screen, white, [
                self.position + pygame.math.Vector2(math.cos(math.radians(self.angle)), -math.sin(math.radians(self.angle))) * 10,
                self.position + pygame.math.Vector2(math.cos(math.radians(self.angle + 160)), -math.sin(math.radians(self.angle + 160))) * 10,
                self.position + pygame.math.Vector2(math.cos(math.radians(self.angle - 160)), -math.sin(math.radians(self.angle - 160))) * 10,
            ])

        # Print rank on screen
        info_text = font.render(f"{self.label} {self.rank}", True, white)
        screen.blit(info_text, (self.position.x + 15, self.position.y - 10))
    
    def get_target(self, POIs):
        target = None
        closest = perception_radius + 1
        self.mode = 0
        for poi in POIs:
            distance = self.position.distance_to(poi.position)
            if distance < perception_radius:
                if distance < closest:          # replace current poi with closer poi
                    closest = distance
                    target = poi
        return target

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
            if neighbour.rank < self.rank:
                steering += neighbour.velocity * 2                              # total velocity
                total += 1 * 2
            elif neighbour.rank == self.rank:
                steering += neighbour.velocity * 1                              # total velocity
                total += 1 * 1
            elif neighbour.rank > self.rank:
                steering += neighbour.velocity * 0.5                              # total velocity
                total += 1 * 0.5
        if total > 0:
            steering = steering/total                                   # avg velocity
            steering = (steering - self.velocity) / dt                  # acceleration needed to match avg velocity at next dt
        return steering

    def cohesion(self, neighbours):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        for neighbour in neighbours:
            if neighbour.rank < self.rank:
                steering += neighbour.position * 2                              # total position
                total += 1 * 2
            elif neighbour.rank == self.rank:
                steering += neighbour.position * 1                              # total position
                total += 1 * 1
            elif neighbour.rank > self.rank:
                steering += neighbour.position * 0.5                              # total position
                total += 1 * 0.5
        if total > 0:
            steering /= total                                           # avg position
            target_pos = steering
            rel_target_pos = target_pos - self.position                                   # Delta p
            steering = 2 * (rel_target_pos - self.velocity * dt) / dt**2      # acceleration needed to arrive at centre of mass at next dt
        
        return steering

    def separation(self, neighbours):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        closest = safe_distance + 1
        for neighbour in neighbours:
            distance = self.position.distance_to(neighbour.position)
            if distance < safe_distance:
                if distance < closest:
                    closest = distance
                away_v = -(neighbour.position - self.position)          # vector pointing away
                away_v.scale_to_length(safe_distance)                   # scaled vector to match safe_dist
                goal_p = neighbour.position + away_v                    # goal position w.r.t. one particular neighbour
                steering += goal_p                                      
                total += 1
        if total > 0:
            steering /= total                                           # avg goal position
            steering -= self.position                                   # Delta p
            steering = 2 * (steering - self.velocity * dt) / dt**2      # acceleration needed to arrive at centre of mass at next dt
        if steering.length() == 0:
            weight_sep = 0
        else:
            weight_sep = (safe_distance - closest) / safe_distance
        return steering, weight_sep

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
    def __init__(self, x, y):
        self.position = pygame.math.Vector2(x,y)
        self.count = 0
    
    def show(self, screen):
        pygame.draw.circle(screen, pink, (self.position.x, self.position.y), POI_radius)
        pygame.draw.circle(screen, red, (self.position.x, self.position.y), 10)

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
            POIs.append(POI(x, y))
        
        # Draw POI
        for poi in POIs:
            poi.update(boids, POIs, screen)
            poi.show(screen)

        # Use ThreadPoolExecutor to update each boid in parallel
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Apply behaviors and update boids concurrently
            futures = [executor.submit(boid.apply_behavior, boids, POIs) for boid in boids]
            concurrent.futures.wait(futures)  # Wait for all boids to complete calculation
            
            new_values = [future.result() for future in futures]
            for boid, (n_rank, n_acceleration) in zip(boids, new_values):
                boid.update(n_rank, n_acceleration)

        # Draw boids
        for boid in boids:
            boid.show_perception(boids, screen)
            boid.show(screen, font=pygame.font.Font(None, 24))

        print(f"Time Step: {ts}")
        ts += 1

        pygame.display.flip()
        # Adjust to see in slow motion. Try 15 or 10
        clock.tick(30)


    pygame.quit()

if __name__ == "__main__":
    main()