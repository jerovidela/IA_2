import tensorflow as tf
from tensorflow.keras.preprocessing.image import load_img, img_to_array #type: ignore
import matplotlib.pyplot as plt
import numpy as np

# 1. Cargamos el modelo
modelo_completo = tf.keras.models.load_model('CNN_model.h5')

# 2. Sub-modelo cortado en la SEGUNDA capa convolucional (Índice 2)
extractor_capa2 = tf.keras.models.Model(
    inputs=modelo_completo.inputs, 
    outputs=modelo_completo.layers[3].output
)

# 3. Cargar la MISMA imagen que usaste antes para poder comparar
# ruta_imagen = "./images/train/right/1090.png" 
ruta_imagen = "./images/train/down/4389.png" 
# ruta_imagen = "./images/train/up/7679.png" 

img = load_img(ruta_imagen, color_mode='grayscale', target_size=(40, 110))
img_array = img_to_array(img)
img_batch = np.expand_dims(img_array, axis=0)

# 4. Predicción (Nos va a devolver 16 mapas)
mapas_capa2 = extractor_capa2.predict(img_batch)

# 5. Visualización de los 16 filtros
plt.figure(figsize=(15, 8))

for i in range(16):
    plt.subplot(4, 4, i + 1)
    mapa = mapas_capa2[0, :, :, i]
    plt.imshow(mapa, cmap='viridis')
    plt.title(f"Filtro {i+1} (Capa 2)")
    plt.axis('off')

plt.tight_layout()
plt.show()