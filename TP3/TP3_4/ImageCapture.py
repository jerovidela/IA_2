import os
import pygame
from pathlib import Path

class ImageCapture():
    def __init__(self):
        # Ya no necesitamos las coordenadas de la ventana (window_left/top)
        self.count = 0
        
        # Aseguramos que existan las carpetas
        for folder in ["up", "down", "right", "live","secuencia_cruda"]:
            Path(f"./images/{folder}/").mkdir(parents=True, exist_ok=True)

    def take_screenshot(self, screen, key):
        self.count += 1
        # Definimos el área de recorte RELATIVA a la ventana del juego (0,0)
        # x=0, y=100, ancho=1100, alto=400 (mantiene tu recorte vertical)
        crop_rect = pygame.Rect(0, 100, 1100, 400)
        
        # .subsurface no copia los píxeles, solo apunta a esa área (es ultra rápido)
        try:
            sub_surface = screen.subsurface(crop_rect)
            pygame.image.save(sub_surface, "./images/{}/{}.png".format(key, self.count))
        except ValueError:
            # Por si el área de recorte se sale de la ventana por algún error
            pygame.image.save(screen, "./images/{}/{}.png".format(key, self.count))

    def capture(self, screen, userInput):
        if userInput[pygame.K_UP]:
            self.take_screenshot(screen, "up")
        elif userInput[pygame.K_DOWN]:
            self.take_screenshot(screen, "down")
        else:
            self.take_screenshot(screen, "right")

    def capture_live(self, screen):
        crop_rect = pygame.Rect(0, 100, 1100, 400)
        try:
            sub_surface = screen.subsurface(crop_rect)
            pygame.image.save(sub_surface, "./images/live/temp.png")
        except ValueError:
            pygame.image.save(screen, "./images/live/temp.png")