import random
import numpy as np
from Dinosaur import Dinosaur

class GeneticAlgorithm:
    def __init__(self, mutation_rate=0.05, elitism_count=15, tournament_size=3):
        self.mutation_rate = mutation_rate
        self.elitism_count = elitism_count
        self.tournament_size = tournament_size

    def next_generation(self, old_population):
        # 1. Ordenar la población muerta por score (de mayor a menor)
        sorted_pop = sorted(old_population, key=lambda x: x.score, reverse=True)
        
        new_population = []
        
        # 2. Elitismo: Conservar a los 15 mejores intactos
        for i in range(self.elitism_count):
            # Clonamos al dinosaurio para no mantener referencias a objetos "muertos"
            elite_dino = Dinosaur(sorted_pop[i].id, sorted_pop[i].color, autoplay=True)
            # Copiamos sus matrices exactas
            elite_dino.W1 = np.copy(sorted_pop[i].W1)
            elite_dino.b1 = np.copy(sorted_pop[i].b1)
            elite_dino.W2 = np.copy(sorted_pop[i].W2)
            elite_dino.b2 = np.copy(sorted_pop[i].b2)
            
            new_population.append(elite_dino)
            
        # 3. Cruzamiento y Mutación para los 95 restantes
        for i in range(self.elitism_count, len(old_population)):
            parent1 = self.select_top_parent(sorted_pop, top_ratio=0.20)
            parent2 = self.select_top_parent(sorted_pop, top_ratio=0.35)
            
            # Crear el hijo
            child_dino = Dinosaur(old_population[i].id, old_population[i].color, autoplay=True)
            
            # Factor de ponderación para la combinación aritmética
            alpha = random.uniform(0, 1) 
            
            def simple_arithmetic_crossover(m1, m2):
                # 1. Aplanar las matrices para tratarlas como un vector unidimensional (cromosoma)
                flat_m1 = m1.flatten()
                flat_m2 = m2.flatten()
                
                # 2. Elegir punto de corte aleatorio
                crossover_point = random.randint(0, len(flat_m1) - 1)
                
                # 3. Copiar la primera parte intacta del Padre 1
                child_flat = np.copy(flat_m1)
                
                # 4. Aplicar la combinación aritmética desde el punto de corte hasta el final
                child_flat[crossover_point:] = alpha * flat_m1[crossover_point:] + (1 - alpha) * flat_m2[crossover_point:]
                
                # 5. Devolver a las dimensiones matriciales originales
                return child_flat.reshape(m1.shape)

            # Aplicar el cruce a cada capa del "cerebro" del dinosaurio
            child_dino.W1 = simple_arithmetic_crossover(parent1.W1, parent2.W1)
            child_dino.b1 = simple_arithmetic_crossover(parent1.b1, parent2.b1)
            child_dino.W2 = simple_arithmetic_crossover(parent1.W2, parent2.W2)
            child_dino.b2 = simple_arithmetic_crossover(parent1.b2, parent2.b2)
            # --- FIN DEL CÓDIGO A AGREGAR / REEMPLAZAR ---
            
            # Aplicar Mutación por Perturbación
            self.mutate(child_dino)
            
            new_population.append(child_dino)
            
        return new_population

    def select_fittest(self, population):
        tournament = random.sample(population, self.tournament_size)
        winner = max(tournament, key=lambda x: x.score)
        return winner

    def select_top_parent(self, sorted_population, top_ratio=0.25):
        """Selecciona un padre con sesgo hacia los mejores individuos."""
        top_count = max(2, int(len(sorted_population) * top_ratio))
        top_candidates = sorted_population[:top_count]
        weights = [top_count - i for i in range(top_count)]
        return random.choices(top_candidates, weights=weights, k=1)[0]


    def mutate(self, child):
        # Función auxiliar para mutar una matriz específica
        def mutate_matrix(matrix):
            # Máscara booleana: True en el 10% de los casos
            mutation_mask = np.random.rand(*matrix.shape) < self.mutation_rate
            # Generar perturbaciones entre -0.1 y 0.1
            perturbation = np.random.uniform(-0.1, 0.1, size=matrix.shape)
            # Sumar la perturbación solo donde la máscara es True
            matrix[mutation_mask] += perturbation[mutation_mask]
            return matrix

        # Aplicar a todas las matrices del hijo
        child.W1 = mutate_matrix(child.W1)
        child.b1 = mutate_matrix(child.b1)
        child.W2 = mutate_matrix(child.W2)
        child.b2 = mutate_matrix(child.b2)