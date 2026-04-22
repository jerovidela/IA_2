import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
from logica_difusa import VariableBorrosa 
from logica_difusa import ControladorDifuso

CONSTANTE_M = 2 # Masa del carro
CONSTANTE_m = 1 # Masa de la pertiga
CONSTANTE_l = 1 # Longitud dela pertiga
CONSIGNA_POS = 0 #Dónde queremos que termine el carro [m]
factor = 10 # Factor de truncamiento para limitar la cantidad de decimales y evitar problemas numéricos

# CONTROLADOR INTERNO: el que busca estabilizar el péndulo
# tiene como input al theta y theta prima y como output la fuerza, lo mismo que antes.

# posicion [rad]
lim_pos_grad = 45  #En grados
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
centros_salida = np.array([50, 25, 0, -25, -50])

# instanciado de ControladorDifuso
controlador = ControladorDifuso(posicion, velocidad, matriz_fam, centros_salida)

#CONTROLADOR EXTERNO: el que busca alcanzar la referencia de pocisión del carro
#Este controlador toma como input la posición lineal y velocidad lineal del carro y devuelve
#como output el ángulo que necesita inclinar al péndulo para alcanzar la posición deseada.

pos_carro = VariableBorrosa(-5, 5, 2000)

pos_carro.f_hombro_izq('MN', -5, -2.5)
pos_carro.f_triangular('PN',-5, -2.5, 0)
pos_carro.f_triangular('Z', -2.5, 0, 2.5)
pos_carro.f_triangular('PP', 0, 2.5, 5)
pos_carro.f_hombro_der('MP', 2.5, 5)

vel_carro = VariableBorrosa(-5, 5, 2000)

vel_carro.f_hombro_izq('MN', -5, -2.5)
vel_carro.f_triangular('PN', -5, -2.5, 0)
vel_carro.f_triangular('Z', -2.5, 0, 2.5)
vel_carro.f_triangular('PP', 0, 2.5, 5)
vel_carro.f_hombro_der('MP', 2.5, 5)

# matriz FAM y centros de salida EXTERNOS (Ángulo deseado en radianes)
matriz_fam_ext = np.array([[4,4,4,3,2], [4,4,3,2,1], [4,3,2,1,0], [3,2,1,0,0], [2,1,0,0,0]])
# MP=10°, PP=5°, Z=0°, PN=-5°, MN=-10°
centros_salida_ext = np.deg2rad([-10, -5, 0, 5, 10]) 

# instanciado de Controlador Externo
controlador_ext = ControladorDifuso(pos_carro, vel_carro, matriz_fam_ext, centros_salida_ext)


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
    
# --- NUEVO: Inicializamos la memoria del motor antes del bucle ---
    fuerza_anterior = 0.0
    # Definimos cuánto puede variar la fuerza (ej: 5000 N/s). 
    # Lo subí un poco respecto al ejemplo anterior porque con un delta_t de 0.001 
    # el motor necesita reaccionar razonablemente rápido para que el péndulo no se caiga.
    max_variacion_fuerza = 1000.0 * delta_t 
    # -----------------------------------------------------------------

    # 1. Simulación matemática completa
    for t in x_tiempo:
        if abs(theta) > np.deg2rad(179) and abs(v_ang) < 0.5:   #Ojo con esto que le pusimos. no es controlador difuso. Solo esta para sacarlo de la quietud de la indeterminación de los 180°
            fuerza_deseada = 100.0
            
        else:
            error_x_carro = x_carro - CONSIGNA_POS
            # 1. El controlador externo mira la pista y pide una inclinación (ej: -8 grados)
            #Este es el ángulo que necesita inclinar al péndulo para mover el carro a la posición del carro deseada
            #El ángulo se mide desde la vertical hacia arriba igual que antes.
            theta_ref_crudo = controlador_ext.inferir(error_x_carro, v_carro)
            
            # 2. El Saturador de Seguridad (por si acaso, aunque los centros ya lo limitan)
            theta_ref = np.clip(theta_ref_crudo, np.deg2rad(-10), np.deg2rad(10))
            #No vamos a dejar que el ángulo de inclinación del péndulo sea mayor que -+10°.
            #En realidad la lógica difusa ya lo devuelve en esos límites, pero por las dudas acotamos nuevamente
            
            # 3. El gran truco: "El Engaño"
            # Calculamos el error. Si theta=0 y nos piden theta_ref=-10, el error es +10.
            # El controlador interno va a creer que el péndulo se está cayendo a +10 grados.
            error_theta = theta - theta_ref
            
            # 4. El Malabarista hace equilibrio basado en esa mentira
            # (OJO: acá usamos el nombre de la instancia original, que se llama 'controlador')
            fuerza_deseada = controlador.inferir(error_theta, v_ang)
        
        # --- NUEVO: Aplicamos los límites físicos del motor ---
        # 1. Calculamos cuánto nos pide saltar el controlador
        salto_fuerza = fuerza_deseada - fuerza_anterior
        
        # 2. Limitamos el salto a la inercia del motor
        salto_fuerza = np.clip(salto_fuerza, -max_variacion_fuerza, max_variacion_fuerza)
        
        # 3. Aplicamos la fuerza real
        fuerza_aplicada = fuerza_anterior + salto_fuerza
        
        # 4. Aplicamos el límite absoluto (saturación máxima de 150 N)
        fuerza_aplicada = np.clip(fuerza_aplicada, -100, 100)
        fuerza_aplicada = np.trunc(fuerza_aplicada * factor) / factor
        
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
        texto_datos.set_text(f"Tiempo: {tiempo_actual:.2f} s\nÁngulo: {np.degrees(theta_actual):.3f}°\nPos. Carro: {x_actual:.1f} m\nFuerza: {fuerza_actual:.3f} N\nVel. Angular: {np.degrees(v_ang):.2f} °/s\nVel. Carro: {v_carro:.2f} m/s\n")

        return barra, texto_datos, carro

    frames_totales = len(x_tiempo) // int(0.02 / delta_t)
    ani = FuncAnimation(fig_anim, update, frames=frames_totales, init_func=init, blit=False, interval=20, repeat=False)

    # =========================
    # FIGURA 1: PÉNDULO (ANGULAR)
    # =========================
    fig_ang, axs_ang = plt.subplots(2, 2, figsize=(12, 8))
    fig_ang.canvas.manager.set_window_title('Análisis Angular (Péndulo)')

    # Posición angular
    axs_ang[0, 0].plot(x_tiempo, np.rad2deg(historial_theta))
    axs_ang[0, 0].set_title("Posición Angular θ (deg)")
    axs_ang[0, 0].grid(True)

    # Velocidad angular
    axs_ang[0, 1].plot(x_tiempo, np.rad2deg(historial_vel_ang))
    axs_ang[0, 1].set_title("Velocidad Angular ω (deg/s)")
    axs_ang[0, 1].grid(True)

    # Aceleración angular
    axs_ang[1, 0].plot(x_tiempo, np.rad2deg(historial_acel_ang))
    axs_ang[1, 0].set_title("Aceleración Angular α (deg/s²)")
    axs_ang[1, 0].grid(True)

    # Fuerza (misma en ambos sistemas)
    axs_ang[1, 1].plot(x_tiempo, historial_fuerza)
    axs_ang[1, 1].set_title("Fuerza Aplicada (N)")
    axs_ang[1, 1].grid(True)

    plt.tight_layout()


    # =========================
    # FIGURA 2: CARRO (LINEAL)
    # =========================
    fig_lin, axs_lin = plt.subplots(2, 2, figsize=(12, 8))
    fig_lin.canvas.manager.set_window_title('Análisis Lineal (Carro)')

    # Posición carro
    axs_lin[0, 0].plot(x_tiempo, historial_x)
    axs_lin[0, 0].set_title("Posición Carro x (m)")
    axs_lin[0, 0].grid(True)

    # Velocidad carro
    axs_lin[0, 1].plot(x_tiempo, historial_vel_carro)
    axs_lin[0, 1].set_title("Velocidad Carro v (m/s)")
    axs_lin[0, 1].grid(True)

    # Aceleración carro
    axs_lin[1, 0].plot(x_tiempo, historial_acel_carro)
    axs_lin[1, 0].set_title("Aceleración Carro a (m/s²)")
    axs_lin[1, 0].grid(True)

    # Fuerza (repetida para comparar)
    axs_lin[1, 1].plot(x_tiempo, historial_fuerza)
    axs_lin[1, 1].set_title("Fuerza Aplicada (N)")
    axs_lin[1, 1].grid(True)

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
# graficar_funciones_pertenencia()
simular_animado(60, 0.01, 180, 0, 0)