import subprocess
try:
    import tensorflow as tf
except ImportError as err:
    subprocess.check_call(['pip', 'install', 'tensorflow'])
    subprocess.check_call(['pip', 'install', 'Pillow'])
    import tensorflow as tf

import pygame
import os
import glob
from NeuralNetwork import NeuralNetwork
import numpy as np
from tensorflow.keras.preprocessing.image import load_img, img_to_array #type: ignore

# Bring images from assets
RUNNING = [os.path.join("Assets/Dino", "DinoRun1.png"),
           os.path.join("Assets/Dino", "DinoRun2.png")]
JUMPING = [os.path.join("Assets/Dino", "DinoJump.png")]
DUCKING = [os.path.join("Assets/Dino", "DinoDuck1.png"),
           os.path.join("Assets/Dino", "DinoDuck2.png")]
CLASSES = ["DUCK", "RIGHT", "JUMP"]

class Dinosaur(NeuralNetwork):
    # Define as global the starting position for the dinosaur
    X_POS = 80
    Y_POS = 310
    Y_POS_DUCK = 340
    JUMP_VEL = 8.5

    def __init__(self, id, mask_color = None, autoplay = False):
        # As 'NeuralNetwork' serves as base class for the dinosaur, start its 'brain'
        super().__init__()
        
        self.id = id
        self.color = mask_color
        self.autoPlay = autoplay
        self.duck_img = self.load_images(DUCKING)
        self.run_img = self.load_images(RUNNING)
        self.jump_img = self.load_images(JUMPING)

        self.resetStatus()
        
        # If a tensorflow model is provided, load it
        model_file = 'CNN_model.h5' 
        if os.path.exists(model_file):         
            # Cargar el modelo si el archivo existe
            self.model = tf.keras.models.load_model(model_file) 
            self.model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

    # Basic state the dinosaur is in when spawning
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

    # Load the image form assets masking it with a layer of the selected color for this dino
    def load_images(self, base_name):
        images = []
        for image_path in base_name:
            result = pygame.image.load(image_path).convert_alpha()

            # Apply the color mask if a color is selected
            if self.color:
                result.fill(self.color, special_flags=pygame.BLEND_ADD)
            images.append(result)

        return images

    # Update the dinosaur's status
    def update(self, userInput):
        # Execute the corresponding actions for the current state
        if self.dino_duck:
            self.duck()
        if self.dino_run:
            self.run()
        if self.dino_jump:
            self.jump()

        if self.step_index >= 10:
            self.step_index = 0

        # Set the next state for the dinosaur. The selection mode depends on the playmode selected for the game.
        if self.autoPlay:
            if userInput == "JUMP" and not self.dino_jump:
                self.dino_duck = False
                self.dino_run = False
                self.dino_jump = True
            elif userInput == "DUCK":
                self.dino_duck = True
                self.dino_run = False
                self.dino_jump = False
            elif not (self.dino_jump or userInput == "DUCK"):
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

        # Avoid cloud-walking
        if not self.dino_jump and self.dino_rect.y < self.Y_POS:
            self.dino_rect.y += 8
            if self.dino_rect.y >= self.Y_POS:
                self.dino_rect.y = self.Y_POS

    def duck(self):
        # Change the image every 5 frames to walk
        self.image = self.duck_img[self.step_index // 5]

        # If we duck on mid-air, fall faster by aumenting rapidly the dinosaur's height until reaching ground
        if (self.dino_rect.y < self.Y_POS):
            self.dino_rect.y += self.JUMP_VEL * 6
            if (self.dino_rect.y >= self.Y_POS_DUCK):
                self.dino_rect.y = self.Y_POS_DUCK
        # Set the ducking position when grounded
        else:
            self.dino_rect = self.image.get_rect()
            self.dino_rect.x = self.X_POS
            self.dino_rect.y = self.Y_POS_DUCK
            self.jump_vel = self.JUMP_VEL

        self.step_index += 1

    def run(self):
        # Change the image every 5 frames to walk
        self.image = self.run_img[self.step_index // 5]

        # Set the running position
        self.dino_rect = self.image.get_rect()
        self.dino_rect.x = self.X_POS
        self.dino_rect.y = self.Y_POS

        self.step_index += 1

    def jump(self):
        # Change the image
        self.image = self.jump_img[0]

        # Reduce the dinosaur's position until the jumping speed is negative; then fall 
        if self.dino_jump:
            self.dino_rect.y -= self.jump_vel * 4
            self.jump_vel -= 0.8

            # Prevent going through the ground
            if self.dino_rect.y >= self.Y_POS:
                self.dino_rect.y = self.Y_POS
                self.dino_jump = False
                self.jump_vel = self.JUMP_VEL

    # Draw the element on screen
    def draw(self, SCREEN):
        SCREEN.blit(self.image, (self.dino_rect.x, self.dino_rect.y))

    # When playing in automatic mode using the tensorflow model, takes a frame and sends it to the model to define the next action
    def predict(self):
        self.autoPlay = True
        
        # Take a screenshot and arrange the image
        # ===================== ARREGLAR TAMAÑO DE IMAGEN, NORMALIZACIÓN Y CANTIDAD DE CLASES PREDICHAS DE SER NECESARIO ===============
        temp_path = "./images/live/temp.png"
        if not os.path.exists(temp_path):
            return
        img = load_img(temp_path, color_mode='grayscale', target_size=(40,110))
        img_array = img_to_array(img)
        # img_array = img_array / 255.0  # Normaliza los valores de píxeles entre 0 y 1
        img_array = np.expand_dims(img_array, axis=0)  # Agrega una dimensión extra para el batch

        # Use the model to make a decision based on the screenshot
        predictions = self.model.predict(img_array)[0] # Obtenemos el array 1D de probabilidades
        
        # predictions tiene este orden (alfabético de tus carpetas):
        # Índice 0: DUCK (Agacharse)
        # Índice 1: RIGHT (Correr/Nada)
        # Índice 2: JUMP (Saltar)

        predicted_class_index = 1 # Por defecto, si dudamos, seguimos corriendo (RIGHT)

        # Umbral de confianza: Solo actuamos si la red está muy segura (> 75%)
        umbral_seguridad = 0.75 

        if predictions[0] > umbral_seguridad:
            predicted_class_index = 0  # DUCK
        elif predictions[2] > umbral_seguridad:
            predicted_class_index = 2  # JUMP
        # ==============================================================================================================================

        # Call the update method with the result
        self.update(CLASSES[predicted_class_index])