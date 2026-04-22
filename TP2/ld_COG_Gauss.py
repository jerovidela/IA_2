import numpy as np
import skfuzzy as fuzz

class VariableBorrosa():
    def __init__(self, val_min, val_max, puntos):
        self.universo = np.linspace(val_min, val_max, puntos) # un universo para cada variable lingüística
        self.conjuntos = {}
    
    def f_triangular(self, conjunto_borroso, a, b, c): # Ej: (MN, a,b,c)
        self.conjuntos[conjunto_borroso] = fuzz.trimf(self.universo, [a, b, c]) # (eje x , terna para armar el triangulo)

    def f_hombro_izq(self, conjunto_borroso, a, b):

        # Definimos el trapecio usando el mínimo del universo para los dos primeros puntos
        self.conjuntos[conjunto_borroso] = fuzz.trapmf(self.universo, [self.universo[0], self.universo[0], a, b])

    def f_hombro_der(self, conjunto_borroso, a, b):

        # Definimos el trapecio usando el máximo del universo para los dos últimos puntos
        self.conjuntos[conjunto_borroso] = fuzz.trapmf(self.universo, [a, b, self.universo[-1], self.universo[-1]])

    def f_gaussiana(self, conjunto_borroso, media, sigma):
        self.conjuntos[conjunto_borroso] = fuzz.gaussmf(self.universo, media, sigma)
        
    def fuzzificar(self, valor_real):
        variable_entrada = {}
        valor_seguro = np.clip(valor_real, self.universo[0], self.universo[-1])
        for conjunto_borroso , valor_y in self.conjuntos.items():
            grado_pertenencia = fuzz.interp_membership(self.universo, valor_y, valor_seguro)

            variable_entrada[conjunto_borroso] = grado_pertenencia 
        return variable_entrada

class ControladorDifuso():
    def __init__(self, var_posicion, var_velocidad, var_salida, matriz_fam):
        self.var_posicion = var_posicion  
        self.var_velocidad = var_velocidad
        self.var_salida = var_salida 
        self.matriz_fam = matriz_fam

    def inferir(self, posicion_act, velocidad_act):
        conjuntos = {'MN': 0, 'PN': 1, 'Z': 2, 'PP': 3, 'MP': 4}
        

        indice_a_nombre = {0: 'MP', 1: 'PP', 2: 'Z', 3: 'PN', 4: 'MN'}
        
        # Inicializamos las activaciones en 0 para cada conjunto
        activacion_salidas = {nombre: 0.0 for nombre in indice_a_nombre.values()}

        # grado de pertenencia de posicion y velocidad actuales
        pertenencia_pos = self.var_posicion.fuzzificar(posicion_act)
        pertenencia_vel = self.var_velocidad.fuzzificar(velocidad_act)
        
        # generacion de reglas 
        for conjunto_pos, valor_pos in pertenencia_pos.items():
            for conjunto_vel, valor_vel in pertenencia_vel.items():
                peso_regla = min(valor_pos, valor_vel) 

                if peso_regla > 0:
                    fila = conjuntos[conjunto_vel]
                    columna = conjuntos[conjunto_pos]
                    
                    indice_salida = self.matriz_fam[fila,columna]
                    nombre_salida = indice_a_nombre[indice_salida]

                    # Guardamos el máximo grado de activación para ese conjunto
                    activacion_salidas[nombre_salida] = max(activacion_salidas[nombre_salida], peso_regla)

        if sum(activacion_salidas.values()) == 0:
            print("Pendulo fuera de rango")
            return 0
        else:

            # Preparamos un arreglo vacío del mismo tamaño que el universo de salida
            agregado = np.zeros_like(self.var_salida.universo)
            
            for nombre_conjunto, grado_activacion in activacion_salidas.items():
                # Cortamos (min) el triángulo de salida original por la altura del grado de activación
                conjunto_cortado = np.fmin(grado_activacion, self.var_salida.conjuntos[nombre_conjunto])
                
                # Superponemos (max) el área de esta regla con las áreas de las demás reglas
                agregado = np.fmax(agregado, conjunto_cortado)
                
            # Calculamos el centro de masa (centroide) del área agregada resultante
            fuerza_cog = fuzz.defuzz(self.var_salida.universo, agregado, 'centroid')
            
            return fuerza_cog