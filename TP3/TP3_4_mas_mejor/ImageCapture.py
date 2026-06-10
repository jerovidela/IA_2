import os
import pygame
from pathlib import Path

# Cada cuántos frames se guarda una imagen de clase "right"
# 1 = todos, 5 = uno de cada cinco (subsampling 5x)
RIGHT_SAMPLE_RATE = 1

class ImageCapture():
    def __init__(self):
        self.count = 0
        self.right_frame_counter = 0  # Contador para subsampling de "right"

        # Aseguramos que existan las carpetas
        for folder in ["up", "down", "right", "live", "secuencia_cruda"]:
            Path(f"./images/{folder}/").mkdir(parents=True, exist_ok=True)

    def take_screenshot(self, screen, label, game_speed):
        """
        Guarda una captura recortada de la ventana del juego.
        Nombre del archivo: {label}_vel{game_speed}_{count:05d}.png
        Ejemplo: up_vel25_00042.png
        """
        self.count += 1
        crop_rect = pygame.Rect(0, 100, 1100, 400)
        filename = f"{label}_vel{game_speed}_{self.count:05d}.png"
        try:
            sub_surface = screen.subsurface(crop_rect)
            pygame.image.save(sub_surface, f"./images/{label}/{filename}")
        except ValueError:
            pygame.image.save(screen, f"./images/{label}/{filename}")

    def capture(self, screen, userInput, game_speed):
        """
        Modo 'c' (captura manual): guarda frames etiquetados según la tecla presionada.
        Las imágenes 'right' se submuestrean cada RIGHT_SAMPLE_RATE frames.
        """
        if userInput[pygame.K_UP]:
            self.take_screenshot(screen, "up", game_speed)
        elif userInput[pygame.K_DOWN]:
            self.take_screenshot(screen, "down", game_speed)
        else:
            # Subsampling: solo guardar 1 de cada RIGHT_SAMPLE_RATE frames right
            self.right_frame_counter += 1
            if self.right_frame_counter >= RIGHT_SAMPLE_RATE:
                self.right_frame_counter = 0
                self.take_screenshot(screen, "right", game_speed)

    def capture_exhibition(self, screen, action, game_speed):
        """
        Modo 'e' (exhibición/genético): guarda frames etiquetados según la acción
        que tomó el genético en este frame.
        Mapeo: JUMP -> up | DUCK -> down | RUN -> right
        Las imágenes 'right' se submuestrean igual que en capture().
        """
        # Mapeo de acciones del genético a nombres de carpeta
        action_to_label = {
            "JUMP": "up",
            "DUCK": "down",
            "RUN":  "right",
        }
        label = action_to_label.get(action, "right")

        if label == "right":
            self.right_frame_counter += 1
            if self.right_frame_counter >= RIGHT_SAMPLE_RATE:
                self.right_frame_counter = 0
                self.take_screenshot(screen, "right", game_speed)
        else:
            self.take_screenshot(screen, label, game_speed)

    def capture_live(self, screen):
        """
        Modo 'a' (inferencia en vivo): guarda el frame actual como temp.png.
        La velocidad del juego se pasa directamente a predict() desde main,
        no se codifica en el nombre del archivo.
        """
        crop_rect = pygame.Rect(0, 100, 1100, 400)
        try:
            sub_surface = screen.subsurface(crop_rect)
            pygame.image.save(sub_surface, "./images/live/temp.png")
        except ValueError:
            pygame.image.save(screen, "./images/live/temp.png")