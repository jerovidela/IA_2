import numpy as np
import scipy as sc
import matplotlib.pyplot as plt

# lectura del dataset
with open('datos.txt', 'r') as f:
    datos_clean = (linea.replace('(', '').replace(')', '') for linea in f)
 
    datos = np.loadtxt(datos_clean, delimiter= ',')
    datos_x = datos[:, 0]
    datos_y = datos[:, 1]

# dimensiones matriciales (m filas, 1 columna)
X = datos_x.reshape(-1, 1)
Y = datos_y.reshape(-1, 1)

# mezclado de indices
indices = np.random.permutation(len(X))

# Calculo del índice de corte para el 80%
corte = int(len(X) * 0.8)

# Separamos los índices para entrenamiento y prueba
indices_train = indices[:corte]
indices_test = indices[corte:]

# matrices finales
X_train, Y_train = X[indices_train], Y[indices_train]
X_test, Y_test = X[indices_test], Y[indices_test]

# clase para definir una capa
class capa_red():
    def __init__(self, n_neuronas, n_conexiones, f_activacion):
        self.f_activacion = f_activacion
        self.bias = (np.random.rand(1, n_neuronas) * 2 - 1)
        self.w = (np.random.rand(n_conexiones, n_neuronas) * 2 - 1) 

# funciones de activacion con su derivada

# para capas ocultas
sigmoide = (lambda x: 1 / (1 + np.e ** (-x)),
            lambda x: x * (1 - x))

leaky_relu = (lambda x: np.where(x > 0, x, 0.01 * x),
              lambda x: np.where(x > 0, 1, 0.01))
            
# para capa de salida
identidad = (lambda x: x,
             lambda x: np.ones_like(x))

f_costo = (lambda Yp, Yr: np.mean((Yp - Yr) ** 2),
            lambda Yp, Yr: (Yp - Yr))

topologia = [1, 10, 5, 1]

def crear_rn(topologia, f_activacion = (identidad, leaky_relu)):
    
    red_neuronal = []

    for l, neuronas in enumerate(topologia[: - 1]):
        
        if l == len(topologia) - 2:    
            red_neuronal.append(capa_red(topologia[l+1],topologia[l],f_activacion[0]))
        
        else:
            red_neuronal.append(capa_red(topologia[l+1],topologia[l],f_activacion[1]))
        
    return red_neuronal

def train(red_neuronal, X_train, Y_train, X_test, Y_test, f_costo, lr = 0.05, epochs = 1000):
    
    errores_train = []
    errores_test = []

    for epoch in range(epochs):
        # forward pass

        current_signal = X_train

        historial = [(None, X_train)] # tiene la forma (z,a). z es none al principio porque no tienen pesos las entradas

        for capa in red_neuronal:

                z = current_signal @ capa.w + capa.bias

                current_signal = capa.f_activacion[0](z)

                historial.append((z,current_signal))  

        # calculo de error

        error_train = f_costo[0](current_signal, Y_train)   # comparamos la ultima señal (salida predicha) vs salida real
        error_test = f_costo[0](predict(red_neuronal, X_test), Y_test)

        errores_train.append(error_train)
        errores_test.append(error_test)

        # back propagation

        delta = []
        
        for l in reversed(range(0, len(red_neuronal))):

            a = historial[l + 1][1]
            if l == len(red_neuronal) - 1:

                delta.insert(0, f_costo[1](a,Y_train) * red_neuronal[l].f_activacion[1](a))

            else:
                
                delta.insert(0, (delta[0] @ red_neuronal[l + 1].w.T) * red_neuronal[l].f_activacion[1](a))

        # gradient descent --- batch gradient descent

        for l in range(len(red_neuronal)):
            
            # entrada actual es la activacion de la capa anterior
            entrada = historial[l][1]

            # cantidad de datos
            m = entrada.shape[0]

            red_neuronal[l].w = red_neuronal[l].w - ((entrada.T @ delta[l])/ m) * lr    # dividimos por m para ajustar el gradiente de pesos

            red_neuronal[l].bias = red_neuronal[l].bias - np.mean(delta[l], axis=0, keepdims=True) * lr

    return errores_train, errores_test

# solo hace forward pass 
def predict(red_neuronal, X):
    current_signal = X
    for capa in red_neuronal:
        z = current_signal @ capa.w + capa.bias
        current_signal = capa.f_activacion[0](z)
    return current_signal


# --- ENTRENAMIENTO ---
mi_red = crear_rn(topologia=[1, 8, 4, 2, 1])

errores_train, errores_test = train(mi_red, X_train, Y_train, X_test, Y_test, f_costo, lr=0.005, epochs=1000)

# --- VISUALIZACIÓN ---

# curva roja con datos que NO pertenecen al dataset
X_curva = np.linspace(np.min(X), np.max(X), 100).reshape(-1, 1)
Y_curva = predict(mi_red, X_curva)

# Le pedimos a la red que prediga los datos de prueba para calcular el error final
Y_pred_test = predict(mi_red, X_test)
error_prueba = f_costo[0](Y_pred_test, Y_test)

plt.figure(figsize=(10, 6))

plt.scatter(X_train, Y_train, s=10, label='Datos de Entrenamiento (80%)', alpha=0.5)

plt.scatter(X_test, Y_test, s=20, label=f'Datos de Prueba (20%) - Error: {error_prueba:.2f}')

plt.plot(X_curva, Y_curva, color='red', linewidth=3, label='Curva de Predicción de la Red')

plt.title('Aproximación de Tendencia con Red Neuronal')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)

plt.show()

# --- GRÁFICO 2: CURVA DE APRENDIZAJE ---
plt.figure(figsize=(10, 6))

# Dibujamos ambas curvas
plt.plot(errores_train, label='Error de Entrenamiento (Train)', color='green', linewidth=2)
plt.plot(errores_test, label='Error de Prueba (Test)', color='violet', linewidth=2)

plt.title('Curvas de pérdida')
plt.xlabel('Épocas')
plt.ylabel('Error Cuadrático Medio (MSE)')

plt.legend()
plt.grid(True)
plt.show()