class Obstacle:
    def __init__(self, image, type, screen_width, game_speed, obstacles):
        self.image = image
        self.type = type
        self.rect = self.image[self.type].get_rect()
        self.rect.x = screen_width
        self.game_speed = game_speed
        self.obstacles = obstacles

    # Update the obstacle's status
    def update(self):
        self.rect.x -= self.game_speed
        if self.rect.x < -self.rect.width:
            self.obstacles.pop()

    # Draw the element on screen
    def draw(self, SCREEN):
        SCREEN.blit(self.image[self.type], self.rect)