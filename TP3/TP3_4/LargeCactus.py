import pygame
import os
import random
from Obstacle import Obstacle

# Bring images from assets
LARGE_CACTUS = [pygame.image.load(os.path.join("Assets/Cactus", "LargeCactus1.png")),
                pygame.image.load(os.path.join("Assets/Cactus", "LargeCactus2.png")),
                pygame.image.load(os.path.join("Assets/Cactus", "LargeCactus3.png"))]

class LargeCactus(Obstacle):
    def __init__(self, screen_width, game_speed, obstacles):
        # Charge the base class with information, select the cactus' amount and the image shown when appears
        self.type = random.randint(0, 2)
        super().__init__(LARGE_CACTUS, self.type, screen_width, game_speed, obstacles)
        self.rect.y = 300