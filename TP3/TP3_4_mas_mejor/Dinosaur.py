import subprocess
try:
    import tensorflow as tf
except ImportError as err:
    subprocess.check_call(['pip', 'install', 'tensorflow'])
    subprocess.check_call(['pip', 'install', 'Pillow'])
    import tensorflow as tf
import pygame
import os
from NeuralNetwork import NeuralNetwork
import numpy as np
from tensorflow.keras.preprocessing.image import load_img, img_to_array  # type: ignore

# Bring images from assets
RUNNING = [os.path.join("Assets/Dino", "DinoRun1.png"),
           os.path.join("Assets/Dino", "DinoRun2.png")]
JUMPING = [os.path.join("Assets/Dino", "DinoJump.png")]
DUCKING = [os.path.join("Assets/Dino", "DinoDuck1.png"),
           os.path.join("Assets/Dino", "DinoDuck2.png")]

# Orden alfabético de carpetas: down=0, right=1, up=2
# Correspondencia: DUCK=down=0, RIGHT=right=1, JUMP=up=2
CLASSES = ["DUCK", "RIGHT", "JUMP"]

# Valor holgado para normalización: cubre el rango real (~155 a 13500 pts)
# y deja margen para partidas futuras más largas sin salirse del rango [0, 1]
GAME_SPEED_MAX = 250.0


class Dinosaur(NeuralNetwork):
    X_POS = 80
    Y_POS = 310
    Y_POS_DUCK = 340
    JUMP_VEL = 8.5

    def __init__(self, id, mask_color=None, autoplay=False):
        super().__init__()

        self.id = id
        self.color = mask_color
        self.autoPlay = autoplay
        self.duck_img = self.load_images(DUCKING)
        self.run_img = self.load_images(RUNNING)
        self.jump_img = self.load_images(JUMPING)
        self.resetStatus()

        # Cargar modelo CNN si existe y compilar una función de inferencia
        # tf.function convierte el forward pass en un grafo compilado,
        # eliminando el overhead de reconstrucción que tiene model.predict()
        model_file = 'SPEED_model.h5'
        self.model = None
        self._infer = None
        if os.path.exists(model_file):
            self.model = tf.keras.models.load_model(model_file)
            self.model.compile(optimizer='adam',
                               loss='categorical_crossentropy',
                               metrics=['accuracy'])
            # Compilar el forward pass como grafo estático (se compila una sola vez
            # en la primera llamada y luego corre a velocidad nativa)
            self._infer = tf.function(self.model, reduce_retracing=True)

    def resetStatus(self):
        self.dino_duck = False
        self.dino_run = True
        self.dino_jump = False
        self.step_index = 0
        self.jump_vel = self.JUMP_VEL
        self.image = self.run_img[0]
        self.dino_rect = self.image.get_rect()
        self.dino_rect.x = self.X_POS
        self.dino_rect.y = self.Y_POS
        self.alive = True
        self.score = 0

    def load_images(self, base_name):
        images = []
        for image_path in base_name:
            result = pygame.image.load(image_path).convert_alpha()
            if self.color:
                result.fill(self.color, special_flags=pygame.BLEND_ADD)
            images.append(result)
        return images

    def update(self, userInput):
        if self.dino_duck:
            self.duck()
        if self.dino_run:
            self.run()
        if self.dino_jump:
            self.jump()
        if self.step_index >= 10:
            self.step_index = 0

        if self.autoPlay:
            if userInput == "JUMP" and not self.dino_jump:
                self.dino_duck = False
                self.dino_run = False
                self.dino_jump = True
            elif userInput == "DUCK":
                self.dino_duck = True
                self.dino_run = False
                self.dino_jump = False
            elif userInput == "RUN" or not (self.dino_jump or userInput == "DUCK"):
                self.dino_duck = False
                self.dino_run = True
                self.dino_jump = False
        else:
            if userInput[pygame.K_UP] and not self.dino_jump:
                self.dino_duck = False
                self.dino_run = False
                self.dino_jump = True
            elif userInput[pygame.K_DOWN]:
                self.dino_duck = True
                self.dino_run = False
                self.dino_jump = False
            elif not (self.dino_jump or userInput[pygame.K_DOWN]):
                self.dino_duck = False
                self.dino_run = True
                self.dino_jump = False

        if not self.dino_jump and self.dino_rect.y < self.Y_POS:
            self.dino_rect.y += 8
            if self.dino_rect.y >= self.Y_POS:
                self.dino_rect.y = self.Y_POS

    def duck(self):
        self.image = self.duck_img[self.step_index // 5]
        if self.dino_rect.y < self.Y_POS:
            self.dino_rect.y += self.JUMP_VEL * 6
            if self.dino_rect.y >= self.Y_POS_DUCK:
                self.dino_rect.y = self.Y_POS_DUCK
        else:
            self.dino_rect = self.image.get_rect()
            self.dino_rect.x = self.X_POS
            self.dino_rect.y = self.Y_POS_DUCK
            self.jump_vel = self.JUMP_VEL
        self.step_index += 1

    def run(self):
        self.image = self.run_img[self.step_index // 5]
        self.dino_rect = self.image.get_rect()
        self.dino_rect.x = self.X_POS
        self.dino_rect.y = self.Y_POS
        self.step_index += 1

    def jump(self):
        self.image = self.jump_img[0]
        if self.dino_jump:
            self.dino_rect.y -= self.jump_vel * 4
            self.jump_vel -= 0.8
            if self.dino_rect.y >= self.Y_POS:
                self.dino_rect.y = self.Y_POS
                self.dino_jump = False
                self.jump_vel = self.JUMP_VEL

    def draw(self, SCREEN):
        SCREEN.blit(self.image, (self.dino_rect.x, self.dino_rect.y))

    def predict(self, game_speed):
        """
        Modo 'a': lee el último frame capturado y lo envía al modelo CNN
        junto con la velocidad normalizada para decidir la próxima acción.

        Usa self._infer (tf.function compilado) en lugar de model.predict()
        para evitar el overhead de Keras y mantener el juego a 30 FPS.
        """
        self.autoPlay = True

        if self._infer is None:
            return

        temp_path = "./images/live/temp.png"
        if not os.path.exists(temp_path):
            return

        # ===================== PREPROCESAMIENTO DE IMAGEN =====================
        img = load_img(temp_path, color_mode='grayscale', target_size=(40, 110))
        img_array = img_to_array(img)
        # La normalización 0-1 la hace la capa Rescaling dentro del modelo
        img_array = np.expand_dims(img_array, axis=0)  # (1, 40, 110, 1)
        # ======================================================================

        # Normalizar la velocidad al rango [0, 1]
        speed_input = np.array([[game_speed / GAME_SPEED_MAX]], dtype=np.float32)  # (1, 1)

        # Inferencia con grafo compilado: sin overhead, velocidad nativa
        predictions = self._infer(
            {"image_input": img_array, "speed_input": speed_input},
            training=False
        ).numpy()[0]

        # Índices: 0=DUCK, 1=RIGHT, 2=JUMP  (orden alfabético de carpetas)
        predicted_class_index = 1  # Por defecto: RIGHT (seguir corriendo)
        umbral_seguridad_up = 0.55
        umbral_seguridad_down = 0.75
        if predictions[0] > umbral_seguridad_down:
            predicted_class_index = 0  # DUCK
        elif predictions[2] > umbral_seguridad_up:
            predicted_class_index = 2  # JUMP

        self.update(CLASSES[predicted_class_index])