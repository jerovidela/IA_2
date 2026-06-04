import tensorflow as tf
from tensorflow.keras.preprocessing.image import load_img, img_to_array #type: ignore
import matplotlib.pyplot as plt
import numpy as np
import os

# 1. Cargamos tu modelo ya entrenado
print("Cargando el modelo...")
modelo_completo = tf.keras.models.load_model('CNN_model.h5')

# 2. Creamos el "Sub-modelo" que se detiene en la primera capa (Conv2D)
# model.layers[1] es la primera capa convolucional que tiene 8 filtros
extractor_caracteristicas = tf.keras.models.Model(
    inputs=modelo_completo.inputs, 
    outputs=modelo_completo.layers[1].output
)

# 3. Cargamos una imagen de prueba 
# ruta_imagen = "./images/train/right/1090.png" 
ruta_imagen = "./images/train/down/4389.png" 
# ruta_imagen = "./images/train/up/7679.png" 

img = load_img(ruta_imagen, color_mode='grayscale', target_size=(40, 110))
img_array = img_to_array(img)
img_batch = np.expand_dims(img_array, axis=0) # Simulamos que es un batch de 1 imagen

# 4. Pasamos la imagen por el sub-modelo
# El resultado tendrá la forma (1, Alto, Ancho, 8) porque tenemos 8 filtros
mapas_caracteristicas = extractor_caracteristicas.predict(img_batch)

# 5. Visualización con Matplotlib
plt.figure(figsize=(15, 6))

# Dibujamos la imagen original redimensionada para comparar
plt.subplot(2, 5, 1)
plt.imshow(img_array[:, :, 0], cmap='gray')
plt.title("Imagen Original (40x110)")
plt.axis('off')

# Dibujamos los 8 mapas de características
for i in range(8):
    plt.subplot(2, 5, i + 2)
    # Extraemos el mapa número 'i'
    mapa = mapas_caracteristicas[0, :, :, i]
    plt.imshow(mapa, cmap='viridis') # Usamos el mapa de color viridis para resaltar contrastes
    plt.title(f"Filtro {i+1}")
    plt.axis('off')

plt.tight_layout()
plt.show()