import pygame
import os
import random
from Obstacle import Obstacle

# Bring images from assets
SMALL_CACTUS = [pygame.image.load(os.path.join("Assets/Cactus", "SmallCactus1.png")),
                pygame.image.load(os.path.join("Assets/Cactus", "SmallCactus2.png")),
                pygame.image.load(os.path.join("Assets/Cactus", "SmallCactus3.png"))]

class SmallCactus(Obstacle):
    def __init__(self, screen_width, game_speed, obstacles):
        # Charge the base class with information, select the cactus' amount and the image shown when appears
        self.type = random.randint(0, 2)
        super().__init__(SMALL_CACTUS, self.type, screen_width, game_speed, obstacles)
        self.rect.y = 325