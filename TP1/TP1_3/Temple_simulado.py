import csv
import sys
import os
import itertools 
import random
import math
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from warehouse.grid import Grid
from pathfinding.astar import AStar
from pathfinding.heuristics import manhattan_distance

from warehouse.entities import Forklift
from simulation.simulation import Simulation 

CHARGING_STATION_POS = (0, 5)
CHARGING_STATION_ID = "CS"

def build_warehouse() -> Grid:
    grid = Grid(width=13, height=11)
    grid.create_charging_station("CS", x=0,  y=5) 
    
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
    """Lee el archivo CSV y transforma los números en IDs (ej: "40" -> "SH40")."""
    ordenes = []
    with open(ruta_archivo, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            orden = [f"SH{int(id_prod):02d}" for id_prod in row if id_prod.strip()]
            ordenes.append(orden)
    return ordenes

def precalcular_distancias(grid: Grid) -> dict[tuple[str, str], float]:
    """
    Calcula la distancia real usando A* entre todos los pares posibles
    (48 estanterías + 1 estación de carga).
    """
    print("Precalculando matriz de distancias con A*.")
    
    nodos = [CHARGING_STATION_ID] + [f"SH{i:02d}" for i in range(1, 49)]
    planner = AStar(grid=grid, heuristic=manhattan_distance)
    matriz_distancias = {}
    
    for nodo_a, nodo_b in itertools.combinations(nodos, 2):
        if nodo_a == CHARGING_STATION_ID:
            celda_a = grid.get_cell(*CHARGING_STATION_POS)
        else:
            celda_a = grid.resolve_goal(nodo_a)
            
        if nodo_b == CHARGING_STATION_ID:
            celda_b = grid.get_cell(*CHARGING_STATION_POS)
        else:
            celda_b = grid.resolve_goal(nodo_b)
            
        ruta = planner.search(celda_a, celda_b)
        distancia = float(len(ruta))
        
        clave = tuple(sorted([nodo_a, nodo_b]))
        matriz_distancias[clave] = distancia

    print(f"¡Listo! Se guardaron {len(matriz_distancias)} rutas óptimas en caché.\n")
    return matriz_distancias

def consultar_distancia(nodo_a: str, nodo_b: str, matriz: dict) -> float:
    """Busca la distancia al instante en O(1)."""
    if nodo_a == nodo_b:
        return 0.0
    clave = tuple(sorted([nodo_a, nodo_b]))
    return matriz[clave]

def calcular_costo_ruta_rapido(ruta: list[str], matriz: dict) -> float:
    """
    Calcula el costo total de la orden sumando las distancias de la matriz.
    """
    distancia_total = 0.0
    nodo_actual = CHARGING_STATION_ID
    
    for producto in ruta:
        distancia_total += consultar_distancia(nodo_actual, producto, matriz)
        nodo_actual = producto
        
    distancia_total += consultar_distancia(nodo_actual, CHARGING_STATION_ID, matriz)
    return distancia_total

def generar_vecino(ruta: list[str]) -> list[str]:
    """Genera un estado vecino intercambiando dos productos de lugar al azar."""
    vecino = ruta.copy()
    if len(vecino) > 1:
        idx1, idx2 = random.sample(range(len(vecino)), 2)
        vecino[idx1], vecino[idx2] = vecino[idx2], vecino[idx1]
    return vecino

def estimar_temperatura_inicial(ruta_base: list[str], matriz: dict, prob_aceptacion: float = 0.8, muestras: int = 100) -> float:
    costo_base = calcular_costo_ruta_rapido(ruta_base, matriz)
    aumentos_energia = []
    
    for _ in range(muestras):
        vecino = generar_vecino(ruta_base)
        costo_vecino = calcular_costo_ruta_rapido(vecino, matriz)
        delta_e = costo_vecino - costo_base
        
        if delta_e > 0:
            aumentos_energia.append(delta_e)
            
    if not aumentos_energia:
        return 100.0 
        
    delta_e_promedio = sum(aumentos_energia) / len(aumentos_energia)
    t0 = -delta_e_promedio / math.log(prob_aceptacion)
    return t0

def optimizar_ruta_ts(ruta_inicial: list[str], matriz: dict, alfa: float = 0.95, temp_final: float = 0.1, iteraciones_por_temp: int = 50) -> tuple[list[str], float]:
    t_actual = estimar_temperatura_inicial(ruta_inicial, matriz)
    ruta_actual = ruta_inicial.copy()
    costo_actual = calcular_costo_ruta_rapido(ruta_actual, matriz)
    
    mejor_ruta = ruta_actual.copy()
    mejor_costo = costo_actual
    
    while t_actual > temp_final:
        for _ in range(iteraciones_por_temp):
            vecino = generar_vecino(ruta_actual)
            costo_vecino = calcular_costo_ruta_rapido(vecino, matriz)
            delta_e = costo_vecino - costo_actual
            
            if delta_e < 0 or random.random() < math.exp(-delta_e / t_actual):
                ruta_actual = vecino
                costo_actual = costo_vecino
                
                if costo_actual < mejor_costo:
                    mejor_ruta = ruta_actual.copy()
                    mejor_costo = costo_actual
                    
        t_actual *= alfa
        
    return mejor_ruta, mejor_costo

def simular_recorrido(ruta: list[str], titulo: str):
    """
    Ejecuta la simulación gráfica de un montacargas siguiendo una ruta específica.
    """
    print(f"\n--- Preparando simulación: {titulo} ---")
    
    grid_sim = build_warehouse()
    
    sim = Simulation(grid=grid_sim, heuristic=manhattan_distance, max_steps=1500)
    
    robot = sim.grid.create_forklift("Robot_Picking", x=0, y=5)
    
    metas_pendientes = list(ruta)
    metas_pendientes.append((0, 5))
    if metas_pendientes:
        meta_actual = metas_pendientes.pop(0)
        sim.assign_goal("Robot_Picking", meta_actual)
        
    try:
        from rendering.renderer import WarehouseRenderer
        renderer = WarehouseRenderer(sim.grid, cell_size=62, fps=5) 
        renderer.init()
    except ImportError:
        print("Error: No se encontró Pygame. Imposible mostrar la simulación gráfica.")
        return

    while sim.timestep < sim.max_steps:
        if renderer.should_quit():
            break
            
        statuses = sim.step()
        renderer.render(sim, statuses)
        
        if robot.current_cell == robot.goal_cell:
            if metas_pendientes:
                meta_actual = metas_pendientes.pop(0)
                sim.assign_goal("Robot_Picking", meta_actual)
            elif sim.all_done():
                print(f"¡Recorrido '{titulo}' completado en {sim.timestep} pasos de tiempo!")
                time.sleep(2)
                break
                
    renderer.quit()


if __name__ == "__main__":

    mi_almacen = build_warehouse()
    matriz_cache = precalcular_distancias(mi_almacen)
    todas_las_ordenes = leer_ordenes('ordenes.csv')
    
    print("\n" + "="*50)
    print("--- INICIANDO TEMPLE SIMULADO (MODO 1: INDIVIDUAL) ---")
    
    orden_prueba = todas_las_ordenes[9].copy()
    costo_inicial_m1 = calcular_costo_ruta_rapido(orden_prueba, matriz_cache)
    
    print(f"Ruta Inicial Aleatoria: {orden_prueba}")
    print(f"Costo Teórico Calculado: {costo_inicial_m1:.1f} pasos")
    
    ruta_optima_m1, costo_optimo_m1 = optimizar_ruta_ts(orden_prueba, matriz_cache)
    
    print(f"\nRuta Óptima encontrada: {ruta_optima_m1}")
    print(f"Costo Teórico Optimizado: {costo_optimo_m1:.1f} pasos")
    mejora_m1 = ((costo_inicial_m1 - costo_optimo_m1) / costo_inicial_m1) * 100
    print(f"Mejora del recorrido: {mejora_m1:.2f}%")

    print("\n=== INICIANDO SIMULACIONES VISUALES ===")
        
    simular_recorrido(orden_prueba, titulo="Ruta Original (Ineficiente)")
        
    simular_recorrido(ruta_optima_m1, titulo="Ruta Optimizada (Temple Simulado)")

    respuesta = input("\n¿Deseas correr la Simulación Montecarlo completa? (s/n): ")
    if respuesta.lower() == 's':
        print("\n" + "="*50)
        print("--- INICIANDO MODO 2: SIMULACIÓN MONTECARLO ---")
        print("Procesando todas las órdenes del CSV...")
        
        NUM_CORRIDAS_POR_ORDEN = 5 
        mejoras_totales_almacen = []
        
        for i, orden_original in enumerate(todas_las_ordenes):
            mejoras_de_esta_orden = []
            
            for corrida in range(NUM_CORRIDAS_POR_ORDEN):
                ruta_aleatoria = orden_original.copy()
                random.shuffle(ruta_aleatoria)
                costo_azar = calcular_costo_ruta_rapido(ruta_aleatoria, matriz_cache)
                
                ruta_optimizada, costo_optimizado = optimizar_ruta_ts(ruta_aleatoria, matriz_cache)
                
                mejora_porcentual = ((costo_azar - costo_optimizado) / costo_azar) * 100
                mejoras_de_esta_orden.append(mejora_porcentual)
                
            promedio_orden = sum(mejoras_de_esta_orden) / NUM_CORRIDAS_POR_ORDEN
            mejoras_totales_almacen.append(promedio_orden)
            
            print(f"Orden {i+1:02d} (Tamaño: {len(orden_original):02d} prod) -> Mejora promedio: {promedio_orden:.2f}%")

        mejora_global = sum(mejoras_totales_almacen) / len(mejoras_totales_almacen)
        print("\n" + "*"*50)
        print(f"RESUMEN FINAL DE OPTIMIZACIÓN DEL ALMACÉN")
        print(f"Órdenes procesadas: {len(todas_las_ordenes)}")
        print(f"Eficiencia promedio del algoritmo: {mejora_global:.2f}%")
        print("*"*50 + "\n")