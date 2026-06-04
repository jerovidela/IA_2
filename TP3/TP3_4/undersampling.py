import os
import random

# Ruta a la carpeta que queremos diezmar
folder_path = "./images/right/"
# Nuestro objetivo para que quede balanceado con 'up' y 'down'
target_count = 5000 

# Obtenemos la lista de todos los archivos en la carpeta
all_files = os.listdir(folder_path)
current_count = len(all_files)

print(f"Archivos encontrados inicialmente: {current_count}")

if current_count > target_count:
    # Calculamos cuántas imágenes nos sobran
    files_to_delete_count = current_count - target_count
    
    # Elegimos aleatoriamente cuáles vamos a borrar sin repetir (sample)
    files_to_delete = random.sample(all_files, files_to_delete_count)
    
    print(f"Borrando {files_to_delete_count} imágenes al azar. Esto puede tardar unos segundos...")
    
    # Recorremos la lista y eliminamos los archivos del disco
    for file_name in files_to_delete:
        file_path = os.path.join(folder_path, file_name)
        os.remove(file_path)
        
    print(f"¡Listo! La carpeta ahora tiene {len(os.listdir(folder_path))} imágenes.")
else:
    print("La carpeta ya tiene la cantidad objetivo o menos. No se borró nada.")