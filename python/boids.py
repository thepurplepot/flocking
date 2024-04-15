import pygame
import random
import math

# Constants
WIDTH, HEIGHT = 800, 600
NUM_BOIDS = 100
BOID_SIZE = 2
MAX_SPEED = 3
MIN_SPEED = 1
VISUAL_RANGE = 40
PROTECTED_RANGE = 8
SEPARATION_FORCE = 0.1
ALIGNMENT_FORCE = 0.1
COHESION_FORCE = 0.001
EDGE_FORCE = 0.2
INITIAL_BIAS = 0.001
MAX_BIAS = 0.01
BIAS_INCREMENT = 0.00004
EDGE_MARGIN = 90

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

class Boid:
    def __init__(self, index, x, y):
        self.index = index
        self.bias_val = INITIAL_BIAS
        self.position = pygame.Vector2(x, y)
        self.velocity = pygame.Vector2(random.uniform(-1, 1), random.uniform(-1, 1)).normalize() * MAX_SPEED

    def update(self, boids):
        position_avg = pygame.Vector2(0, 0)
        velocity_avg = pygame.Vector2(0, 0)
        separation = pygame.Vector2(0, 0)
        alignment = pygame.Vector2(0, 0)
        cohesion = pygame.Vector2(0, 0)
        count = 0

        for boid in boids:
            if boid.index != self.index:
                distance = self.position.distance_to(boid.position)
                if distance < VISUAL_RANGE:
                    count += 1
                    position_avg += boid.position
                    velocity_avg += boid.velocity
                    if distance < PROTECTED_RANGE:
                        diff = self.position - boid.position
                        separation += diff
        if count > 0:
            position_avg /= count
            velocity_avg /= count
            alignment = (velocity_avg - self.velocity) * ALIGNMENT_FORCE
            cohesion = (position_avg - self.position) * COHESION_FORCE
            separation *= SEPARATION_FORCE
        
        edge_force = self.edge_avoidance()
        self.velocity += separation + alignment + cohesion + edge_force
        self.update_bias()
        self.bias()
        self.velocity.scale_to_length(MAX_SPEED) # TODO add min speed
        self.position += self.velocity

    
    def edge_avoidance(self):
        edge_force = pygame.Vector2(0, 0)
        if self.position.x < EDGE_MARGIN:
            edge_force.x = EDGE_FORCE
        elif self.position.x > WIDTH - EDGE_MARGIN:
            edge_force.x = -EDGE_FORCE
        if self.position.y < EDGE_MARGIN:
            edge_force.y = EDGE_FORCE
        elif self.position.y > HEIGHT - EDGE_MARGIN:
            edge_force.y = -EDGE_FORCE
        return edge_force
    
    def bias(self):
        if self.index % 2 == 0:
            self.velocity.x = (1-self.bias_val)*self.velocity.x + self.bias_val
            self.velocity.y = (1-self.bias_val)*self.velocity.y + self.bias_val
        else:
            self.velocity.x = (1-self.bias_val)*self.velocity.x - self.bias_val
            self.velocity.y = (1-self.bias_val)*self.velocity.y - self.bias_val

    def update_bias(self):
        if self.index % 2 == 0:
            if self.velocity.x > 0:
                self.bias_val = min(MAX_BIAS, self.bias_val + BIAS_INCREMENT)
            else:
                self.bias_val = max(BIAS_INCREMENT, self.bias_val - BIAS_INCREMENT)
        else:
            if self.velocity.x < 0:
                self.bias_val = min(MAX_BIAS, self.bias_val + BIAS_INCREMENT)
            else:
                self.bias_val = max(BIAS_INCREMENT, self.bias_val - BIAS_INCREMENT)

    def draw(self, screen):
        pygame.draw.circle(screen, WHITE, (int(self.position.x), int(self.position.y)), BOID_SIZE)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Boids Flocking Algorithm")
    clock = pygame.time.Clock()

    boids = [Boid(i, random.randint(0, WIDTH), random.randint(0, HEIGHT)) for i in range(NUM_BOIDS)]

    running = True
    while running:
        screen.fill(BLACK)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for boid in boids:
            boid.update(boids)
            boid.draw(screen)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
