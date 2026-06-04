import tensorflow as tf
import os
import random
import shutil
import matplotlib.pyplot as plt

# Rutas de las carpetas
source_dir = "images"
train_dir = os.path.join(source_dir, "train")
test_dir = os.path.join(source_dir, "test")
classes = ["up", "down", "right"]

train_ratio = 0.8
batch_size = 32
image_size = (40, 110)
input_shape = image_size + (1,)

# ========================== LÓGICA DE DISTRIBUCIÓN ==========================
# Comprobamos si la distribución ya se hizo verificando si existe la carpeta train/up
necesita_distribucion = False
if not os.path.exists(os.path.join(train_dir, classes[0])):
    necesita_distribucion = True

if necesita_distribucion:
    print("Primera ejecución: Creando carpetas y distribuyendo imágenes...")
    
    # Crea los directorios limpios
    for d in [train_dir, test_dir]:
        for c in classes:
            os.makedirs(os.path.join(d, c), exist_ok=True)

    for class_name in classes:
        source_class_dir = os.path.join(source_dir, class_name)
        
        # Filtramos para agarrar solo archivos
        images = [f for f in os.listdir(source_class_dir) if os.path.isfile(os.path.join(source_class_dir, f))]
        
        # Ordenamos y aplicamos una semilla para que el proceso sea replicable
        images.sort()
        random.seed(42) 
        random.shuffle(images)
        
        num_train_images = int(len(images) * train_ratio)
        
        # Copiar para entrenamiento
        for img_name in images[:num_train_images]:
            src = os.path.join(source_class_dir, img_name)
            dst = os.path.join(train_dir, class_name, img_name)
            shutil.copy(src, dst)
                
        # Copiar para prueba
        for img_name in images[num_train_images:]:
            src = os.path.join(source_class_dir, img_name)
            dst = os.path.join(test_dir, class_name, img_name)
            shutil.copy(src, dst)
            
    print("Distribución completada con éxito.")
else:
    print("Las carpetas 'train' y 'test' ya existen. Omitiendo la distribución física.")
# ============================================================================

print("Configurando datasets optimizados...")

# Carga de datos apuntando a las carpetas FÍSICAS que acabas de crear/verificar
train_dataset = tf.keras.utils.image_dataset_from_directory(
    train_dir, # Apuntamos a la carpeta train física
    image_size=image_size,
    batch_size=batch_size,
    color_mode='grayscale',
    label_mode='categorical'
)

validation_dataset = tf.keras.utils.image_dataset_from_directory(
    test_dir, # Apuntamos a la carpeta test física
    image_size=image_size,
    batch_size=batch_size,
    color_mode='grayscale',
    label_mode='categorical'
)

# Caché y Prefetching
AUTOTUNE = tf.data.AUTOTUNE
train_dataset = train_dataset.cache().prefetch(buffer_size=AUTOTUNE)
validation_dataset = validation_dataset.cache().prefetch(buffer_size=AUTOTUNE)

# ========================== CONSTRUIR EL MODELO OPTIMIZADO ==========================================
model = tf.keras.models.Sequential([
    # ATENCIÓN: Se normalizan los píxeles (0-1) aquí mismo dentro de la arquitectura
    tf.keras.layers.Rescaling(1./255, input_shape=input_shape),
    
    tf.keras.layers.Conv2D(8, (3, 3), activation='relu'),
    tf.keras.layers.MaxPooling2D((2, 2)),
    
    tf.keras.layers.Conv2D(16, (3, 3), activation='relu'),
    tf.keras.layers.MaxPooling2D((2, 2)),
    
    tf.keras.layers.Conv2D(32, (3, 3), activation='relu'),
    tf.keras.layers.MaxPooling2D((2, 2)),
    
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(128, activation='relu'),
    tf.keras.layers.Dropout(0.5),
    tf.keras.layers.Dense(len(classes), activation='softmax')
])
# ==========================================================================================

model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

print("Iniciando entrenamiento...")
history = model.fit(train_dataset, epochs=50, validation_data=validation_dataset)

model.save('Nuevo_modelo.h5')
print("Modelo guardado exitosamente.")

# ========================== GRÁFICA DE LOSS ==========================================
plt.figure(figsize=(10, 5))
plt.plot(history.history['loss'], label='Loss de Entrenamiento', marker='o')
plt.plot(history.history['val_loss'], label='Loss de Validación', marker='o')
plt.title('Evolución de la Función de Pérdida (Loss)')
plt.xlabel('Época (Epoch)')
plt.ylabel('Loss (Categorical Crossentropy)')
plt.legend()
plt.grid(True)

# Guardar la gráfica como archivo evita congelamientos en WSL
plt.savefig('Nuevo_modelo.png')
print("Gráfica guardada exitosamente como 'grafica_loss.png'.")