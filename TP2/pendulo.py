import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
from logica_difusa import VariableBorrosa, ControladorDifuso

CONSTANTE_M = 2 # Masa del carro
CONSTANTE_m = 1 # Masa de la pertiga
CONSTANTE_l = 1 # Longitud dela pertiga

# instanciado de variables lingüísticas de entrada
# posicion [rad]
lim_pos_grad = 180  #En grados
lim_pos_rad = np.deg2rad(lim_pos_grad) #Convertido a radianes
posicion = VariableBorrosa(-lim_pos_rad, lim_pos_rad, 2000)

posicion.f_hombro_izq('MN', np.deg2rad(-45), np.deg2rad(-22.5))
posicion.f_triangular('PN', np.deg2rad(-45), np.deg2rad(-22.5), np.deg2rad(0))
posicion.f_triangular('Z',  np.deg2rad(-22.5), np.deg2rad(0),   np.deg2rad(22.5))
posicion.f_triangular('PP', np.deg2rad(0.0),   np.deg2rad(22.5),  np.deg2rad(45))
posicion.f_hombro_der('MP', np.deg2rad(22.5),  np.deg2rad(45))

# velocidad [rad/s]
lim_vel_grados = 300 # grados por segundo
lim_vel_rad = np.deg2rad(lim_vel_grados)
velocidad = VariableBorrosa(-lim_vel_rad, lim_vel_rad, 2000)

velocidad.f_hombro_izq('MN', np.deg2rad(-300),  np.deg2rad(-150))
velocidad.f_triangular('PN', np.deg2rad(-300), np.deg2rad(-150), np.deg2rad(0))
velocidad.f_triangular('Z',  np.deg2rad(-150), np.deg2rad(0),    np.deg2rad(150))
velocidad.f_triangular('PP', np.deg2rad(0),    np.deg2rad(150),  np.deg2rad(300))
velocidad.f_hombro_der('MP', np.deg2rad(150),  np.deg2rad(300))

# matriz FAM y centros de salida
matriz_fam = np.array([[4,4,4,3,2], [4,4,3,2,1], [4,3,2,1,0], [3,2,1,0,0], [2,1,0,0,0]])
centros_salida = np.array([100, 50, 0, -50, -100])

# instanciado de ControladorDifuso
controlador = ControladorDifuso(posicion, velocidad, matriz_fam, centros_salida)

# Calcula la aceleracion en el siguiente instante de tiempo dado el angulo y la velocidad angular actual, y la fuerza ejercida
# Calcula la aceleracion angular (theta_ddot)
def calcula_aceleracion(theta, v, f):
    numerador = 9.80665 * np.sin(theta) + np.cos(theta) * ((-f - CONSTANTE_m * CONSTANTE_l * np.power(v, 2) * np.sin(theta)) / (CONSTANTE_M + CONSTANTE_m))
    denominador = CONSTANTE_l * (4/3 - (CONSTANTE_m * np.power(np.cos(theta), 2) / (CONSTANTE_M + CONSTANTE_m)))
    return numerador / denominador

def calcula_aceleracion_carro(theta, v_ang, a_ang, f):
    termino_inercia = CONSTANTE_m * CONSTANTE_l * a_ang * np.cos(theta)
    termino_centrifugo = CONSTANTE_m * CONSTANTE_l * np.power(v_ang, 2) * np.sin(theta)
    return (f - termino_inercia + termino_centrifugo) / (CONSTANTE_M + CONSTANTE_m)

def simular_animado(t_max, delta_t, theta_0, v_0, a_0):
    theta = (theta_0 * np.pi) / 180
    v_ang = v_0
    a_ang = a_0
    
    # Nuevas variables de estado para el carro
    x_carro = 0.0
    v_carro = 0.0

    historial_theta = []
    historial_x = [] # Guardar posición histórica del carro
    historial_fuerza = []
    historial_vel_ang = []
    historial_vel_carro = []
    historial_acel_ang = []
    historial_acel_carro = []
    x_tiempo = np.arange(0, t_max, delta_t)
    
#Inicializamos la memoria del motor antes del bucle ---
    fuerza_anterior = 0.0
    # el motor necesita reaccionar razonablemente rápido para que el péndulo no se caiga.
    #max_variacion_fuerza = 500.0 * delta_t
    max_variacion_fuerza = 1000000 
    # -----------------------------------------------------------------

    # 1. Simulación matemática completa
    for t in x_tiempo:
        if abs(theta) > np.deg2rad(179) and abs(v_ang) < 0.5:   #Ojo con esto que le pusimos. no es controlador difuso. Solo esta para sacarlo de la quietud de la indeterminación de los 180°
            fuerza_deseada = 100.0
            
        else:
            fuerza_deseada = controlador.inferir(theta, v_ang)
        
       #Aplicamos los límites físicos del motor ---
        # 1. Calculamos cuánto nos pide saltar el controlador
        salto_fuerza = fuerza_deseada - fuerza_anterior
        
        # 2. Limitamos el salto a la inercia del motor
        salto_fuerza = np.clip(salto_fuerza, -max_variacion_fuerza, max_variacion_fuerza)
        
        # 3. Aplicamos la fuerza real
        fuerza_aplicada = fuerza_anterior + salto_fuerza
        
        # 4. Aplicamos el límite absoluto (saturación máxima de 150 N)
        fuerza_aplicada = np.clip(fuerza_aplicada, -100, 100)
        
        # 5. Guardamos para el próximo ciclo
        fuerza_anterior = fuerza_aplicada
        # ------------------------------------------------------
        
        # Dinámica del sistema
        a_ang = calcula_aceleracion(theta, v_ang, fuerza_aplicada)
        a_carro = calcula_aceleracion_carro(theta, v_ang, a_ang, fuerza_aplicada)
        
        # Integración de Euler para el Péndulo
        v_ang = v_ang + a_ang * delta_t
        theta = theta + v_ang * delta_t + a_ang * np.power(delta_t, 2) / 2
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        
        # Integración de Euler para el Carro
        v_carro = v_carro + a_carro * delta_t
        x_carro = x_carro + v_carro * delta_t + a_carro * np.power(delta_t, 2) / 2
        
        historial_theta.append(theta)
        historial_x.append(x_carro)
        historial_fuerza.append(fuerza_aplicada)

        historial_vel_ang.append(v_ang)
        historial_vel_carro.append(v_carro)
        historial_acel_ang.append(a_ang)
        historial_acel_carro.append(a_carro)

    # 2. Configurar la figura de la animación
    fig_anim, ax_anim = plt.subplots(figsize=(10, 6))
    ax_anim.set_aspect('equal')
    ax_anim.grid(True, linestyle='--')
    ax_anim.set_title("Estabilización Dinámica del Péndulo Invertido")
    ax_anim.axhline(0, color='black', linewidth=2) # Suelo

    carro = patches.Rectangle((0, 0), 0.6, 0.3, fc='gray', ec='black')
    ax_anim.add_patch(carro)
    barra, = ax_anim.plot([], [], 'o-', lw=5, color='sienna', markersize=10, markerfacecolor='black')
    texto_datos = ax_anim.text(0, 0, '', transform=ax_anim.transAxes, fontsize=11, bbox=dict(facecolor='white', alpha=0.8))

    def init():
        barra.set_data([], [])
        texto_datos.set_text('')
        return barra, texto_datos, carro

    def update(frame):
        idx = frame * int(0.02 / delta_t) 
        if idx >= len(x_tiempo):
            idx = len(x_tiempo) - 1

        theta_actual = historial_theta[idx]
        x_actual = historial_x[idx]
        fuerza_actual = historial_fuerza[idx]
        x_actual = historial_x[idx]
        tiempo_actual = x_tiempo[idx]
        v_ang = historial_vel_ang[idx]
        v_carro = historial_vel_carro[idx]

        # Actualizar cinemática con la posición del carro acoplada
        px = x_actual + CONSTANTE_l * np.sin(theta_actual)
        py = CONSTANTE_l * np.cos(theta_actual)

        barra.set_data([x_actual, px], [0, py])
        carro.set_xy((x_actual - 0.3, -0.15)) # Centrar carro en su posición X
        
        # La cámara sigue al carro horizontalmente
        ax_anim.set_xlim(x_actual - 2, x_actual + 2)
        ax_anim.set_ylim(-1, 2)
        
        # Posicionar el texto fijo en la esquina de la pantalla
        texto_datos.set_position((0.02, 0.7))
        texto_datos.set_text(f"Tiempo: {tiempo_actual:.2f} s\nÁngulo: {np.degrees(theta_actual):.6f}°\nPos. Carro: {x_actual:.1f} m\nFuerza: {fuerza_actual:.6f} N\nVel. Angular: {np.degrees(v_ang):.4f} °/s\nVel. Carro: {v_carro:.4f} m/s\n")

        return barra, texto_datos, carro

    frames_totales = len(x_tiempo) // int(0.02 / delta_t)
    ani = FuncAnimation(fig_anim, update, frames=frames_totales, init_func=init, blit=False, interval=20, repeat=False)
    plt.show()
    fig_graf, ((ax_pos, ax_vel), (ax_force, ax_acc)) = plt.subplots(2, 2, figsize=(12, 8))
    fig_graf.canvas.manager.set_window_title('Análisis Físico de la Simulación')

    # Gráfico de Posiciones
    ax_pos.plot(x_tiempo, np.rad2deg(historial_theta), label="θ (deg)", color='blue')
    ax_pos.plot(x_tiempo, historial_x, label="x carro (m)", color='orange')
    ax_pos.set_title("Posición")
    ax_pos.set_xlabel("Tiempo (s)")
    ax_pos.grid(True)
    ax_pos.legend()

    # Gráfico de Velocidades
    ax_vel.plot(x_tiempo, np.rad2deg(historial_vel_ang), label="ω (deg/s)", color='green')
    ax_vel.plot(x_tiempo, historial_vel_carro, label="v carro (m/s)", color='red')
    ax_vel.set_title("Velocidad")
    ax_vel.set_xlabel("Tiempo (s)")
    ax_vel.grid(True)
    ax_vel.legend()

    # Gráfico de Fuerza
    ax_force.plot(x_tiempo, historial_fuerza, label="F (N)", color='purple')
    ax_force.set_title("Esfuerzo de Control (Fuerza Aplicada)")
    ax_force.set_xlabel("Tiempo (s)")
    ax_force.grid(True)
    ax_force.legend()

    # Gráfico de Aceleraciones
    ax_acc.plot(x_tiempo, np.rad2deg(historial_acel_ang), label="α (deg/s²)", color='cyan')
    ax_acc.plot(x_tiempo, historial_acel_carro, label="a carro (m/s²)", color='magenta')
    ax_acc.set_title("Aceleración")
    ax_acc.set_xlabel("Tiempo (s)")
    ax_acc.grid(True)
    ax_acc.legend()

    plt.tight_layout()
    plt.show()
def graficar_funciones_pertenencia():
# Creamos una figura con dos subgráficos (uno arriba del otro)
    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(9, 7))
    fig.canvas.manager.set_window_title('Análisis de Variables Lingüísticas')

# 1. Extraer y graficar las conjuntos de Posición
    for etiqueta, valores_y in posicion.conjuntos.items():
        # CONVERSIÓN AQUÍ: Pasamos posicion.universo a grados solo para el gráfico
        ax_pos.plot(np.rad2deg(posicion.universo), valores_y, label=etiqueta, linewidth=2.5)

    ax_pos.set_title("Funciones de Pertenencia: Posición (θ)", fontsize=12, weight='bold')
    ax_pos.set_xlabel("Grados")
    ax_pos.set_ylabel("Grado de pertenencia (μ)")
    ax_pos.grid(True, linestyle='--', alpha=0.7)
    ax_pos.legend(loc='upper right')

    # 2. Extraer y graficar las conjuntos de Velocidad
    for etiqueta, valores_y in velocidad.conjuntos.items():
        # CONVERSIÓN AQUÍ: Pasamos velocidad.universo a grados solo para el gráfico
        ax_vel.plot(np.rad2deg(velocidad.universo), valores_y, label=etiqueta, linewidth=2.5)

    ax_vel.set_title("Funciones de Pertenencia: Velocidad Angular (dθ/dt)", fontsize=12, weight='bold')
    ax_vel.set_xlabel("Grados / segundo")
    ax_vel.set_ylabel("Grado de pertenencia (μ)")
    ax_vel.grid(True, linestyle='--', alpha=0.7)
    ax_vel.legend(loc='upper right')

    plt.tight_layout()
    plt.show() # Esta línea detiene el código hasta que cierres la ventana

# EJECUCIÓN
graficar_funciones_pertenencia()
simular_animado(10, 0.01, 80, 0, 0)