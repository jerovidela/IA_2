import pygame
import os
from Obstacle import Obstacle

# Bring images from assets
BIRD = [pygame.image.load(os.path.join("Assets/Bird", "Bird1.png")),
        pygame.image.load(os.path.join("Assets/Bird", "Bird2.png"))]

class Bird(Obstacle):
    def __init__(self, screen_width, game_speed, obstacles):
        # Charge the base class with information, select the bird's height and the image shown when appears
        super().__init__(BIRD, 0, screen_width, game_speed, obstacles)
        self.rect.y = 230
        self.index = 0

    # Draw the element on screen
    def draw(self, SCREEN):
        if self.index >= 9:
            self.index = 0
        # Change the image every 5 frames to make the bird 'flap'
        SCREEN.blit(self.image[self.index//5], self.rect)
        self.index += 1