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

    def fuzzificar(self, valor_real):
        variable_entrada = {}
        valor_seguro = np.clip(valor_real, self.universo[0], self.universo[-1])

        for conjunto_borroso , valor_y in self.conjuntos.items():
            grado_pertenencia = fuzz.interp_membership(self.universo, valor_y, valor_seguro)

            variable_entrada[conjunto_borroso] = grado_pertenencia 
        return variable_entrada

class ControladorDifuso():
    def __init__(self, var_posicion, var_velocidad, matriz_fam, centros_salida):
        # instancias de la clase VariableBorrosa()
        self.var_posicion = var_posicion  
        self.var_velocidad = var_velocidad

        self.matriz_fam = matriz_fam
        self.centros_salida = centros_salida

    def inferir(self, posicion_act, velocidad_act):
        conjuntos = {'MN': 0, 'PN': 1, 'Z': 2, 'PP': 3, 'MP': 4}
        activacion_salidas = np.zeros(5)

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
                    conjunto_salida = self.matriz_fam[fila,columna]

                    activacion_salidas[conjunto_salida] = max(activacion_salidas[conjunto_salida], peso_regla)
                    # [0,0,0,0,0.66]
                    # cada uno de los INDICES de activacion_salida corresponde a un conjunto difuso de la variable de salida 
                    # cada valor almacenado en cada indice de activacion_salida corresponde al máximo grado de pertenencia ---> 
                    # para el conjunto difuso asociado de la variable de salida dados los valores nítidos de las variables de entrada

        if np.sum(activacion_salidas) == 0:
            print("Pendulo fuera de rango")
            return 0
        else:
            fuerza = np.dot(activacion_salidas, self.centros_salida)/np.sum(activacion_salidas)
            return fuerza