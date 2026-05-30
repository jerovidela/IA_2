import csv
import os
import matplotlib.pyplot as plt

def plot_evolution_data():
    file_path = 'evolution_metrics_20260517_225138.csv'
    
    if not os.path.isfile(file_path):
        print(f"Error: El archivo '{file_path}' no existe. Ejecute primero el simulador para generar datos.")
        return

    generations = []
    max_scores = []
    avg_scores = []

    # Extraer los datos del CSV
    with open(file_path, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            generations.append(int(row['Generacion']))
            max_scores.append(float(row['Max_Score']))
            avg_scores.append(float(row['Avg_Score']))

    if not generations:
        print("El archivo CSV está vacío.")
        return

    # Estructuración de la gráfica
    plt.figure(figsize=(10, 6))
    
    # Curva del individuo élite
    plt.plot(generations, max_scores, label='Puntaje Máximo (Élite)', color='blue', linestyle='-', marker='o', markersize=4)
    
    # Curva de convergencia poblacional
    plt.plot(generations, avg_scores, label='Puntaje Promedio', color='red', linestyle='--', marker='x', markersize=4)
    
    plt.title('Curva de Aprendizaje - Algoritmo Genético')
    plt.xlabel('Generación')
    plt.ylabel('Puntuación (Score)')
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.7)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_evolution_data()