import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras.preprocessing.image import load_img, img_to_array #type: ignore

# 1. Cargamos tu modelo ya entrenado (Corregí el nombre al tuyo)
print("Cargando modelo...")
model = tf.keras.models.load_model('CNN_model.h5')

# 2. Identificar dinámicamente la última capa convolucional
last_conv_layer = [layer for layer in model.layers if isinstance(layer, tf.keras.layers.Conv2D)][-1]
last_conv_index = model.layers.index(last_conv_layer)

# ================= LA SOLUCIÓN: DIVIDIR EL MODELO EN DOS =================
# Modelo A (Backbone): Toma la imagen y devuelve solo los mapas espaciales
extractor_modelo = tf.keras.models.Model(inputs=model.inputs, outputs=last_conv_layer.output)

# Modelo B (Clasificador): Toma los mapas, los aplasta y toma la decisión final
clasificador_input = tf.keras.Input(shape=last_conv_layer.output.shape[1:])
x = clasificador_input
# Reconstruimos la parte final pasando las capas una por una
for layer in model.layers[last_conv_index + 1:]:
    x = layer(x)
clasificador_modelo = tf.keras.models.Model(inputs=clasificador_input, outputs=x)
# ===========================================================================

# 3. Cargamos y preprocesamos la imagen
ruta_imagen = "./images/train/right/7024.png" 
img = load_img(ruta_imagen, color_mode='grayscale', target_size=(40, 110))
img_array = img_to_array(img)
img_batch = np.expand_dims(img_array, axis=0)

# 4. Calculamos los Gradientes explícitamente
with tf.GradientTape() as tape:
    # Paso 1: Pasamos la imagen por el Ojo
    mapas_caracteristicas = extractor_modelo(img_batch)
    
    # ¡LA LÍNEA MÁGICA!: Forzamos a TensorFlow a observar este puente intermedio
    tape.watch(mapas_caracteristicas)
    
    # Paso 2: Pasamos los mapas por el Cerebro para decidir
    preds = clasificador_modelo(mapas_caracteristicas)
    pred_index = tf.argmax(preds[0]) 
    class_channel = preds[:, pred_index] 

# Ahora la derivada NUNCA será None porque el puente está forzado a grabarse
grads = tape.gradient(class_channel, mapas_caracteristicas)

# 5. Promediamos y armamos el mapa de calor
pooled_grads = tf.reduce_mean(grads, axis=(0, 1, 2))
mapas_caracteristicas = mapas_caracteristicas[0]
heatmap = mapas_caracteristicas @ pooled_grads[..., tf.newaxis]
heatmap = tf.squeeze(heatmap)

# Descartamos negativos (ReLU) y normalizamos (0 a 1)
heatmap = tf.maximum(heatmap, 0) / tf.math.reduce_max(heatmap)
heatmap = heatmap.numpy()

# 6. Redimensionamos al tamaño original
heatmap_resized = tf.image.resize(heatmap[..., tf.newaxis], (40, 110)).numpy().squeeze()

nombres_clases = ["DUCK", "RIGHT", "JUMP"]
clase_predicha = nombres_clases[pred_index.numpy()]

# 7. Visualización
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.imshow(img_array[:, :, 0], cmap='gray')
plt.title("Imagen Original")
plt.axis('off')

plt.subplot(1, 2, 2)
plt.imshow(img_array[:, :, 0], cmap='gray') 
plt.imshow(heatmap_resized, cmap='jet', alpha=0.5) # Jet genera el clásico Azul-Verde-Rojo
plt.title(f"Grad-CAM (Predicción: {clase_predicha})")
plt.axis('off')

plt.tight_layout()
plt.show()