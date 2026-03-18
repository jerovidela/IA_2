import csv
import sys
import os
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.insert(0, os.path.dirname(__file__))

from warehouse.grid import Grid
from pathfinding.astar import AStar
from pathfinding.heuristics import manhattan_distance

CHARGING_STATION_POS = (0, 5)
CHARGING_STATION_ID = "CS"

def build_warehouse() -> Grid:
    grid = Grid(width=13, height=11)
    grid.create_charging_station(CHARGING_STATION_ID, x=0,  y=5) 
    
    block_positions = [
        (2, 1), (6, 1), (10, 1),
        (2, 6), (6, 6), (10, 6)
    ]
    shelf_id = 1
    for block_x, block_y in block_positions:
        for dy in range(4):
            for dx in range(2):
                grid.create_shelf(
                    f"SH{shelf_id:02d}",
                    x=block_x + dx,
                    y=block_y + dy
                )
                shelf_id += 1
    return grid

def leer_ordenes(ruta_archivo: str) -> list[list[str]]:
    """Lee el archivo CSV (reutilizado del ejercicio anterior)."""
    ordenes = []
    with open(ruta_archivo, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            orden = [f"SH{int(id_prod):02d}" for id_prod in row if id_prod.strip()]
            ordenes.append(orden)
    return ordenes

def calcular_frecuencias(ordenes: list[list[str]]) -> dict[str, int]:
    """
    Cuenta cuántas veces aparece cada producto en todo el historial.
    Asegura que los 48 productos existan en el diccionario, incluso si tienen 0 pedidos.
    """
    frecuencias = {f"SH{i:02d}": 0 for i in range(1, 49)}
    
    for orden in ordenes:
        for producto in orden:
            if producto in frecuencias:
                frecuencias[producto] += 1
                
    return frecuencias

def precalcular_distancias_a_base(grid: Grid) -> dict[str, float]:
    """
    A diferencia del Temple Simulado (que medía todo contra todo), 
    aquí solo nos importa la distancia desde cada estante físico hasta la Estación de Carga.
    """
    print("Mapeando distancias desde la estación de carga a los estantes...")
    planner = AStar(grid=grid, heuristic=manhattan_distance)
    distancias = {}
    
    celda_base = grid.get_cell(*CHARGING_STATION_POS)
    
    for i in range(1, 49):
        id_estante = f"SH{i:02d}"
        celda_estante = grid.resolve_goal(id_estante)
        
        ruta = planner.search(celda_base, celda_estante)
        distancias[id_estante] = float(len(ruta))
        
    return distancias

def calcular_fitness(cromosoma: list[str], frecuencias: dict[str, int], distancias_base: dict[str, float]) -> float:
    """
    Evalúa qué tan bueno es un diseño de almacén.
    cromosoma: Lista de 48 productos. El índice representa el estante físico.
               Ej: cromosoma[0] es el producto guardado físicamente en el estante SH01.
    """
    costo_total = 0.0
    
    for indice, producto in enumerate(cromosoma):
        estante_fisico = f"SH{indice + 1:02d}"
        frecuencia_producto = frecuencias[producto]
        distancia_del_estante = distancias_base[estante_fisico]
        costo_total += frecuencia_producto * distancia_del_estante
        
    return costo_total

def graficar_mapa_calor(cromosoma: list[str], frecuencias: dict[str, int], titulo: str = "Mapa de Calor del Almacén"):
    """
    Dibuja el almacén en una cuadrícula de 13x11 y pinta las estanterías 
    según la frecuencia histórica de los productos asignados a ellas.
    """
    fig, ax = plt.subplots(figsize=(10, 8))
    
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 11)
    ax.invert_yaxis()
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5, color='grey')
    ax.set_xticks(range(14))
    ax.set_yticks(range(12))
    
    max_freq = max(frecuencias.values()) if frecuencias else 1
    cmap = plt.get_cmap('Reds')

    cs_rect = patches.Rectangle((0, 5), 1, 1, facecolor='gold', edgecolor='black', linewidth=2)
    ax.add_patch(cs_rect)
    ax.text(0.5, 5.5, "C", ha='center', va='center', fontweight='bold', fontsize=16)
    
    block_positions = [
        (2, 1), (6, 1), (10, 1),
        (2, 6), (6, 6), (10, 6)
    ]
    
    indice_estante = 0
    for block_x, block_y in block_positions:
        for dy in range(4):
            for dx in range(2):
                x = block_x + dx
                y = block_y + dy
                
                producto = cromosoma[indice_estante]
                frecuencia = frecuencias[producto]
                
                intensidad = (frecuencia / max_freq) * 0.9 + 0.1 
                color = cmap(intensidad)
                
                rect = patches.Rectangle((x, y), 1, 1, facecolor=color, edgecolor='black')
                ax.add_patch(rect)
                
                prod_num = producto.replace("SH", "")
                ax.text(x+0.5, y+0.5, f"P{prod_num}\n({frecuencia})", ha='center', va='center', fontsize=8, 
                        color='white' if intensidad > 0.6 else 'black')
                
                indice_estante += 1

    ax.set_title(titulo, fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.show()

def generar_poblacion_inicial(tam_poblacion: int) -> list[list[str]]:
    """Crea N individuos (almacenes) con los productos mezclados al azar."""
    poblacion = []
    cromosoma_base = [f"SH{i:02d}" for i in range(1, 49)]
    
    for _ in range(tam_poblacion):
        individuo = cromosoma_base.copy()
        random.shuffle(individuo)
        poblacion.append(individuo)
        
    return poblacion

def seleccion_torneo(poblacion: list[list[str]], fitness_scores: list[float], k: int = 3) -> list[str]:
    """Elige k individuos al azar y devuelve el que tenga el mejor (menor) puntaje."""
    indices_torneo = random.sample(range(len(poblacion)), k)
    
    mejor_indice = indices_torneo[0]
    mejor_fitness = fitness_scores[mejor_indice]
    
    for idx in indices_torneo[1:]:
        if fitness_scores[idx] < mejor_fitness:
            mejor_fitness = fitness_scores[idx]
            mejor_indice = idx
            
    return poblacion[mejor_indice]

def cruzar_ox1(padre: list[str], madre: list[str]) -> list[str]:
    """
    Operador Order Crossover (OX1). 
    Copia una franja del padre y rellena con los genes de la madre en orden.
    """
    size = len(padre)
    hijo = [""] * size
    inicio, fin = sorted(random.sample(range(size), 2))
    hijo[inicio:fin+1] = padre[inicio:fin+1]
    productos_en_hijo = set(hijo[inicio:fin+1])
    productos_faltantes = [prod for prod in madre if prod not in productos_en_hijo]
    idx_faltantes = 0
    for i in range(size):
        if hijo[i] == "":
            hijo[i] = productos_faltantes[idx_faltantes]
            idx_faltantes += 1
            
    return hijo

def mutar(cromosoma: list[str], prob_mutacion: float = 0.05):
    """Intercambia dos productos de lugar según una probabilidad (in-place)."""
    if random.random() < prob_mutacion:
        idx1, idx2 = random.sample(range(len(cromosoma)), 2)
        cromosoma[idx1], cromosoma[idx2] = cromosoma[idx2], cromosoma[idx1]

def ejecutar_ag(frecuencias: dict, distancias: dict, tam_poblacion=100, generaciones=200, prob_mutacion=0.1, elitismo=2):
    """Orquesta todos los módulos para evolucionar el almacén."""
    print(f"\nIniciando Algoritmo Genético (Pop: {tam_poblacion}, Gen: {generaciones})...")
    
    poblacion = generar_poblacion_inicial(tam_poblacion)
    mejor_historico = poblacion[0].copy()
    mejor_fitness_historico = float('inf')
    
    historial_fitness = []
    
    for generacion in range(generaciones):
        fitness_scores = [calcular_fitness(ind, frecuencias, distancias) for ind in poblacion]
        min_fitness = min(fitness_scores)
        mejor_idx = fitness_scores.index(min_fitness)
        
        if min_fitness < mejor_fitness_historico:
            mejor_fitness_historico = min_fitness
            mejor_historico = poblacion[mejor_idx].copy()
            
        historial_fitness.append(mejor_fitness_historico)
    
        if generacion % 50 == 0 or generacion == generaciones - 1:
            print(f"Generación {generacion:03d} | Mejor Fitness: {mejor_fitness_historico:.1f}")
            
        nueva_poblacion = []
        
        indices_ordenados = sorted(range(tam_poblacion), key=lambda i: fitness_scores[i])
        for i in range(elitismo):
            nueva_poblacion.append(poblacion[indices_ordenados[i]].copy())
            
        while len(nueva_poblacion) < tam_poblacion:
            padre = seleccion_torneo(poblacion, fitness_scores)
            madre = seleccion_torneo(poblacion, fitness_scores)
            hijo = cruzar_ox1(padre, madre)
            mutar(hijo, prob_mutacion)
            nueva_poblacion.append(hijo)
            
        poblacion = nueva_poblacion
        
    print("¡Evolución terminada!")
    return mejor_historico, mejor_fitness_historico, historial_fitness

if __name__ == "__main__":
    mi_almacen = build_warehouse()
    todas_las_ordenes = leer_ordenes('ordenes.csv')
    frecuencias_historicas = calcular_frecuencias(todas_las_ordenes)
    distancias_almacen = precalcular_distancias_a_base(mi_almacen)
    almacen_inicial = [f"SH{i:02d}" for i in range(1, 49)]
    costo_inicial = calcular_fitness(almacen_inicial, frecuencias_historicas, distancias_almacen)
    
    almacen_optimo, costo_optimo, curva_evolucion = ejecutar_ag(
        frecuencias=frecuencias_historicas, 
        distancias=distancias_almacen,
        tam_poblacion=100, 
        generaciones=300, 
        prob_mutacion=0.1, 
        elitismo=2
    )
    
    mejora = ((costo_inicial - costo_optimo) / costo_inicial) * 100
    print(f"\n--- RESULTADOS FINALES ---")
    print(f"Costo del almacén original inicial: {costo_inicial:.1f}")
    print(f"Costo del almacén optimizado por AG: {costo_optimo:.1f}")
    print(f"Mejora estructural lograda: {mejora:.2f}%\n")

    print("Mostrando Curva de Evolución (Cerrá la ventana para continuar con los mapas)...")
    plt.figure(figsize=(8, 5))
    plt.plot(curva_evolucion, color='blue', linewidth=2)
    plt.title("Curva de Evolución del Algoritmo Genético", fontsize=14, fontweight='bold')
    plt.xlabel("Generación", fontsize=12)
    plt.ylabel("Mejor Fitness (Costo de Distancia)", fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()
    
    print("Mostrando Almacén Original (Cerrá la ventana para ver el resultado evolucionado)")
    graficar_mapa_calor(almacen_inicial, frecuencias_historicas, titulo=f"Almacén Original Inicial (Fitness: {costo_inicial:.0f})")
    
    print("Mostrando Almacén Evolucionado...")
    graficar_mapa_calor(almacen_optimo, frecuencias_historicas, titulo=f"Almacén Evolucionado por AG (Fitness: {costo_optimo:.0f})")