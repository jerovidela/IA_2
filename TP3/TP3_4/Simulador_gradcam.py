import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import os
from tensorflow.keras.preprocessing.image import load_img, img_to_array #type: ignore

# 1. Configuración de carpetas
input_dir = "./images/secuencia_cruda/"
output_dir = "./images/secuencia_gradcam/"
os.makedirs(output_dir, exist_ok=True)

# 2. Cargamos el modelo
print("Cargando modelo...")
model = tf.keras.models.load_model('ABCNN_model.h5')

# 3. Separación del modelo (para evitar el error NoneType)
last_conv_layer = [layer for layer in model.layers if isinstance(layer, tf.keras.layers.Conv2D)][-1]
last_conv_index = model.layers.index(last_conv_layer)

extractor_modelo = tf.keras.models.Model(inputs=model.inputs, outputs=last_conv_layer.output)

clasificador_input = tf.keras.Input(shape=last_conv_layer.output.shape[1:])
x = clasificador_input
for layer in model.layers[last_conv_index + 1:]:
    x = layer(x)
clasificador_modelo = tf.keras.models.Model(inputs=clasificador_input, outputs=x)

nombres_clases = ["DUCK", "RIGHT", "JUMP"]

# 4. Obtener la lista de imágenes ordenadas numéricamente
# Buscamos archivos .png y los ordenamos por el número en el nombre
archivos = [f for f in os.listdir(input_dir) if f.endswith('.png')]
archivos_ordenados = sorted(archivos, key=lambda x: int(x.split('.')[0]))

print(f"Comenzando el procesamiento de {len(archivos_ordenados)} imágenes...")

# 5. Bucle de procesamiento en lote
for index, nombre_archivo in enumerate(archivos_ordenados):
    ruta_imagen = os.path.join(input_dir, nombre_archivo)
    
    # Preprocesamiento
    img = load_img(ruta_imagen, color_mode='grayscale', target_size=(40, 110))
    img_array = img_to_array(img)
    img_batch = np.expand_dims(img_array, axis=0)

    # Magia Matemática: Grad-CAM
    with tf.GradientTape() as tape:
        mapas_caracteristicas = extractor_modelo(img_batch)
        tape.watch(mapas_caracteristicas)
        preds = clasificador_modelo(mapas_caracteristicas)
        pred_index = tf.argmax(preds[0]) 
        class_channel = preds[:, pred_index] 

    grads = tape.gradient(class_channel, mapas_caracteristicas)
    pooled_grads = tf.reduce_mean(grads, axis=(0, 1, 2))
    mapas_locales = mapas_caracteristicas[0]
    
    heatmap = mapas_locales @ pooled_grads[..., tf.newaxis]
    heatmap = tf.squeeze(heatmap)
    heatmap = tf.maximum(heatmap, 0) / (tf.math.reduce_max(heatmap) + 1e-10) # 1e-10 evita división por cero
    heatmap = heatmap.numpy()

    heatmap_resized = tf.image.resize(heatmap[..., tf.newaxis], (40, 110)).numpy().squeeze()
    clase_predicha = nombres_clases[pred_index.numpy()]

    # Visualización y guardado
    plt.figure(figsize=(10, 4))
    
    # Imagen de fondo con el mapa de calor superpuesto
    plt.imshow(img_array[:, :, 0], cmap='gray') 
    plt.imshow(heatmap_resized, cmap='jet', alpha=0.5) 
    plt.title(f"Frame {nombre_archivo.split('.')[0]} | Predicción IA: {clase_predicha}")
    plt.axis('off')
    
    # Guardamos la imagen sin bordes blancos innecesarios
    ruta_salida = os.path.join(output_dir, nombre_archivo)
    plt.tight_layout()
    plt.savefig(ruta_salida, dpi=100, bbox_inches='tight')
    
    # CERRAR LA FIGURA ES VITAL PARA NO QUEDARSE SIN MEMORIA RAM
    plt.close()

    # Imprimir progreso en la consola
    if (index + 1) % 50 == 0:
        print(f"Procesados {index + 1} de {len(archivos_ordenados)} frames...")

print(f"\n¡Proceso finalizado! Las imágenes están listas en la carpeta '{output_dir}'.")