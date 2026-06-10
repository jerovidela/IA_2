import os

# ================= 0. APAGAR WARNINGS =================
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
os.environ['ABSL_LOGGING_MIN_LOG_LEVEL'] = 'info'

import tensorflow as tf
import numpy as np
import random
import shutil
import re
import matplotlib.pyplot as plt
import warnings

warnings.filterwarnings('ignore')
tf.get_logger().setLevel('ERROR')

SEED = 55
tf.keras.utils.set_random_seed(SEED)

# ========================== CONFIGURACIÓN ==========================
source_dir  = "images"
train_dir   = os.path.join(source_dir, "train")
test_dir    = os.path.join(source_dir, "test")
classes     = ["down", "right", "up"]   # orden alfabético = índices 0,1,2

train_ratio = 0.8
batch_size  = 32
image_size  = (40, 110)
input_shape = image_size + (1,)          # canal único (escala de grises)

# Debe coincidir con GAME_SPEED_MAX en Dinosaur.py
# Valor holgado: cubre el rango real y deja margen para partidas futuras más largas
GAME_SPEED_MAX = 250.0
# ===================================================================


# ========================== DISTRIBUCIÓN TRAIN/TEST ==========================
def distribute_images():
    """
    Copia las imágenes de images/{clase}/ a images/train/{clase}/ e images/test/{clase}/
    preservando el nombre original (que incluye la velocidad).
    Solo se ejecuta si la distribución no se hizo todavía.
    """
    if os.path.exists(os.path.join(train_dir, classes[0])):
        print("Las carpetas 'train' y 'test' ya existen. Omitiendo la distribución.")
        return

    print("Primera ejecución: distribuyendo imágenes en train/test...")
    for d in [train_dir, test_dir]:
        for c in classes:
            os.makedirs(os.path.join(d, c), exist_ok=True)

    for class_name in classes:
        source_class_dir = os.path.join(source_dir, class_name)
        images = [f for f in os.listdir(source_class_dir)
                  if os.path.isfile(os.path.join(source_class_dir, f))]
        images.sort()
        random.seed(42)
        random.shuffle(images)

        n_train = int(len(images) * train_ratio)
        for img_name in images[:n_train]:
            shutil.copy(os.path.join(source_class_dir, img_name),
                        os.path.join(train_dir, class_name, img_name))
        for img_name in images[n_train:]:
            shutil.copy(os.path.join(source_class_dir, img_name),
                        os.path.join(test_dir, class_name, img_name))

    print("Distribución completada.")

distribute_images()


# ========================== CONSTRUCCIÓN DEL DATASET tf.data ==========================
def build_samples(data_dir):
    """
    Recorre data_dir/clase/ y devuelve tres listas paralelas:
      - paths:  ruta completa a cada imagen (string)
      - speeds: velocidad normalizada extraída del nombre (float32)
      - labels: one-hot vector (lista de floats)

    El parseo de la velocidad se hace aquí en Python puro (una sola vez),
    antes de construir el pipeline tf.data, para que el caché almacene
    tensores ya procesados y no haya trabajo Python dentro del grafo.
    """
    paths, speeds, labels = [], [], []
    class_index = {c: i for i, c in enumerate(classes)}
    pattern = re.compile(r'_vel(\d+)_')

    for class_name in classes:
        class_dir = os.path.join(data_dir, class_name)
        if not os.path.isdir(class_dir):
            continue
        label_idx = class_index[class_name]
        oh = [0.0] * len(classes)
        oh[label_idx] = 1.0

        for fname in os.listdir(class_dir):
            if not fname.lower().endswith('.png'):
                continue
            match = pattern.search(fname)
            speed = (int(match.group(1)) / GAME_SPEED_MAX) if match else 0.0
            paths.append(os.path.join(class_dir, fname))
            speeds.append(speed)
            labels.append(oh)

    return paths, speeds, labels


def load_image(path):
    """Lee, decodifica y redimensiona una imagen PNG en escala de grises."""
    raw   = tf.io.read_file(path)
    img   = tf.image.decode_png(raw, channels=1)          # (H, W, 1), uint8
    img   = tf.image.resize(img, image_size)               # (40, 110, 1), float32
    return img


def make_dataset(data_dir, shuffle):
    """
    Construye el pipeline tf.data completo para un split (train o test).

    Flujo de datos:
      1. Dataset de (path, speed, label) — todo en RAM desde el inicio
      2. .map(load_image) — lee cada PNG del disco y lo decodifica (paralelizado)
      3. .cache()         — almacena los tensores decodificados en RAM
                           (el disco se lee UNA sola vez, en la epoch 1)
      4. .shuffle()       — mezcla aleatoria DENTRO de la RAM (solo en train)
      5. .batch()         — agrupa en batches de 32
      6. .prefetch()      — la CPU prepara el batch N+1 mientras la GPU procesa el N
    """
    paths, speeds, labels = build_samples(data_dir)
    n = len(paths)

    # Dataset base: tres tensores 1D en RAM
    path_ds  = tf.data.Dataset.from_tensor_slices(paths)
    speed_ds = tf.data.Dataset.from_tensor_slices(
                   tf.constant(speeds, dtype=tf.float32)[:, tf.newaxis])  # (N,1)
    label_ds = tf.data.Dataset.from_tensor_slices(
                   tf.constant(labels, dtype=tf.float32))                 # (N,3)

    # Leer imágenes en paralelo (num_parallel_calls=AUTOTUNE usa todos los núcleos)
    img_ds = path_ds.map(load_image, num_parallel_calls=tf.data.AUTOTUNE)

    # Combinar imagen + velocidad como tupla de entradas, con su etiqueta
    dataset = tf.data.Dataset.zip(((img_ds, speed_ds), label_ds))

    # Caché en RAM: tras la primera epoch, ningún acceso a disco
    dataset = dataset.cache()

    if shuffle:
        # buffer_size=n garantiza shuffle uniforme sobre todo el dataset
        dataset = dataset.shuffle(buffer_size=n, seed=SEED,
                                  reshuffle_each_iteration=True)

    dataset = dataset.batch(batch_size)
    dataset = dataset.prefetch(buffer_size=tf.data.AUTOTUNE)

    return dataset, n


print("Configurando datasets...")
train_ds, n_train = make_dataset(train_dir, shuffle=True)
val_ds,   n_val   = make_dataset(test_dir,  shuffle=False)
print(f"  Train: {n_train} imágenes  |  Validation: {n_val} imágenes")


# ========================== ARQUITECTURA DE ENTRADA MIXTA ==========================
#
#   Entrada 1: imagen (40 x 110 x 1)   -> rama CNN
#   Entrada 2: velocidad normalizada (1,) -> rama densa pequeña
#   Ambas ramas se concatenan antes de la capa de salida.
#
#   Flujo por la red (por cada muestra del batch):
#     imagen  -> Rescaling -> Conv2D(8)  -> MaxPool
#                          -> Conv2D(16) -> MaxPool
#                          -> Conv2D(32) -> MaxPool
#                          -> Flatten -> Dense(64) -> Dropout
#                                                          \
#     velocidad -> Dense(8)                                 Concatenate(72) -> Dense(3) -> softmax
#
# ==================================================================================

# --- Rama imagen (CNN) ---
img_input = tf.keras.Input(shape=input_shape, name="image_input")

x = tf.keras.layers.Rescaling(1./255)(img_input)   # normaliza píxeles a [0,1]

x = tf.keras.layers.Conv2D(8,  (3, 3), activation='relu')(x)
x = tf.keras.layers.MaxPooling2D((2, 2))(x)

x = tf.keras.layers.Conv2D(16, (3, 3), activation='relu')(x)
x = tf.keras.layers.MaxPooling2D((2, 2))(x)

x = tf.keras.layers.Conv2D(32, (3, 3), activation='relu')(x)
x = tf.keras.layers.MaxPooling2D((2, 2))(x)

x = tf.keras.layers.Flatten()(x)
x = tf.keras.layers.Dense(64, activation='relu')(x)
x = tf.keras.layers.Dropout(0.5)(x)

# --- Rama velocidad (densa) ---
speed_input = tf.keras.Input(shape=(1,), name="speed_input")
s = tf.keras.layers.Dense(8, activation='relu')(speed_input)

# --- Fusión ---
combined = tf.keras.layers.Concatenate()([x, s])
output   = tf.keras.layers.Dense(len(classes), activation='softmax')(combined)

model = tf.keras.Model(inputs=[img_input, speed_input], outputs=output)
# ===================================================================================

model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

model.summary()

# ========================== CALLBACKS ==========================
reduce_lr = tf.keras.callbacks.ReduceLROnPlateau(
    monitor='val_loss',
    factor=0.5,
    patience=2,
    min_lr=1e-6,
    verbose=0
)

print("\nIniciando entrenamiento...")
history = model.fit(
    train_ds,
    epochs=15,
    validation_data=val_ds,
    callbacks=[reduce_lr]
)

model.save('SPEED_model.h5')
print("Modelo guardado como 'SPEED_model.h5'.")

# ========================== GRÁFICA DE LOSS ==========================
plt.figure(figsize=(10, 5))
plt.plot(history.history['loss'],     label='Loss de Entrenamiento', marker='o')
plt.plot(history.history['val_loss'], label='Loss de Validación',    marker='o')
plt.title('Evolución de la Función de Pérdida (Loss)')
plt.xlabel('Época (Epoch)')
plt.ylabel('Loss (Categorical Crossentropy)')
plt.legend()
plt.grid(True)
plt.savefig('SPEED_model.png')
print("Gráfica guardada como 'SPEED_model.png'.")