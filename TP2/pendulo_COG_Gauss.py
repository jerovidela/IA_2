import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
from ld_COG_Gauss import VariableBorrosa, ControladorDifuso

CONSTANTE_M = 2 # Masa del carro
CONSTANTE_m = 1 # Masa de la pertiga
CONSTANTE_l = 1 # Longitud dela pertiga

# instanciado de variables lingüísticas de entrada

# 1. POSICIÓN [rad]
lim_pos_grad = 180
lim_pos_rad = np.deg2rad(lim_pos_grad)
posicion = VariableBorrosa(-lim_pos_rad, lim_pos_rad, 2000)

# Sigma de ~10 grados para un buen solapamiento
sigma_pos = np.deg2rad(10) 
posicion.f_gaussiana('MN', np.deg2rad(-45), sigma_pos)
posicion.f_gaussiana('PN', np.deg2rad(-22.5), sigma_pos)
posicion.f_gaussiana('Z',  np.deg2rad(0), sigma_pos)
posicion.f_gaussiana('PP', np.deg2rad(22.5), sigma_pos)

# Extremo derecho suave (Hombro S)
posicion.f_hombro_suave_der('MP', np.deg2rad(22.5), np.deg2rad(45))

# 2. VELOCIDAD [rad/s]
lim_vel_grados = 300 
lim_vel_rad = np.deg2rad(lim_vel_grados)
velocidad = VariableBorrosa(-lim_vel_rad, lim_vel_rad, 2000)

# Sigma de ~65 grados por segundo para un buen solapamiento
sigma_vel = np.deg2rad(65)
velocidad.f_gaussiana('MN', np.deg2rad(-300), sigma_vel)
velocidad.f_gaussiana('PN', np.deg2rad(-150), sigma_vel)
velocidad.f_gaussiana('Z',  np.deg2rad(0), sigma_vel)
velocidad.f_gaussiana('PP', np.deg2rad(150), sigma_vel)
velocidad.f_gaussiana('MP', np.deg2rad(300), sigma_vel)

# matriz FAM
matriz_fam = np.array([[4,4,4,3,2], [4,4,3,2,1], [3,3,2,1,1], [3,2,1,0,0], [2,1,0,0,0]])

# 3. SALIDA: FUERZA APLICADA [N]
lim_fuerza = 50 
fuerza = VariableBorrosa(-lim_fuerza, lim_fuerza, 2000)

# Sigma de 20 Newtons
sigma_fuerza = 10.0
fuerza.f_gaussiana('MN', -100, sigma_fuerza)
fuerza.f_gaussiana('PN', -50, sigma_fuerza)
fuerza.f_gaussiana('Z',  0, sigma_fuerza)
fuerza.f_gaussiana('PP', 50, sigma_fuerza)
fuerza.f_gaussiana('MP', 100, sigma_fuerza)

# instanciado de ControladorDifuso
controlador = ControladorDifuso(posicion, velocidad, fuerza, matriz_fam)
# Calcula la aceleracion angular (theta_ddot)
def calcula_aceleracion(theta, v, f):
    numerador = 9.80665 * np.sin(theta) + np.cos(theta) * ((-f - CONSTANTE_m * CONSTANTE_l * np.power(v, 2) * np.sin(theta)) / (CONSTANTE_M + CONSTANTE_m))
    denominador = CONSTANTE_l * (4/3 - (CONSTANTE_m * np.power(np.cos(theta), 2) / (CONSTANTE_M + CONSTANTE_m)))
    return numerador / denominador

# NUEVO: Calcula la aceleracion lineal del carro (x_ddot)
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
    
# Inicializamos la memoria del motor antes del bucle ---
    fuerza_anterior = 0.0
    # el motor necesita reaccionar razonablemente rápido para que el péndulo no se caiga.
    max_variacion_fuerza = 2000.0 * delta_t
    #max_variacion_fuerza = 2000.0 * delta_t
    # -----------------------------------------------------------------

    # 1. Simulación matemática completa
    for t in x_tiempo:
 
        fuerza_deseada = controlador.inferir(theta, v_ang)
        
        # Aplicamos los límites físicos del motor ---
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
     # =========================
    # FIGURA 1: PÉNDULO (ANGULAR)
    # =========================
    fig_ang, axs_ang = plt.subplots(2, 2, figsize=(12, 8))
    fig_ang.canvas.manager.set_window_title('Análisis Angular (Péndulo)')

    # Posición angular
    axs_ang[0, 0].plot(x_tiempo, np.rad2deg(historial_theta), color='blue', linewidth=1.5)
    axs_ang[0, 0].set_title("Posición Angular θ (deg)")
    axs_ang[0, 0].grid(True, linestyle='--', alpha=0.7)

    # Velocidad angular
    axs_ang[0, 1].plot(x_tiempo, np.rad2deg(historial_vel_ang), color='green', linewidth=1.5)
    axs_ang[0, 1].set_title("Velocidad Angular ω (deg/s)")
    axs_ang[0, 1].grid(True, linestyle='--', alpha=0.7)

    # Aceleración angular
    axs_ang[1, 0].plot(x_tiempo, np.rad2deg(historial_acel_ang), color='cyan', linewidth=1.5)
    axs_ang[1, 0].set_title("Aceleración Angular α (deg/s²)")
    axs_ang[1, 0].grid(True, linestyle='--', alpha=0.7)

    # Fuerza (misma en ambos sistemas)
    axs_ang[1, 1].plot(x_tiempo, historial_fuerza, color='purple', linewidth=1.5)
    axs_ang[1, 1].set_title("Fuerza Aplicada (N)")
    axs_ang[1, 1].grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()

    # =========================
    # FIGURA 2: CARRO (LINEAL)
    # =========================
    fig_lin, axs_lin = plt.subplots(2, 2, figsize=(12, 8))
    fig_lin.canvas.manager.set_window_title('Análisis Lineal (Carro)')

    # Posición carro
    axs_lin[0, 0].plot(x_tiempo, historial_x, color='orange', linewidth=1.5)
    axs_lin[0, 0].set_title("Posición Carro x (m)")
    axs_lin[0, 0].grid(True, linestyle='--', alpha=0.7)
    axs_lin[0, 0].legend()

    # Velocidad carro
    axs_lin[0, 1].plot(x_tiempo, historial_vel_carro, color='red', linewidth=1.5)
    axs_lin[0, 1].set_title("Velocidad Carro v (m/s)")
    axs_lin[0, 1].grid(True, linestyle='--', alpha=0.7)

    # Aceleración carro
    axs_lin[1, 0].plot(x_tiempo, historial_acel_carro, color='magenta', linewidth=1.5)
    axs_lin[1, 0].set_title("Aceleración Carro a (m/s²)")
    axs_lin[1, 0].grid(True, linestyle='--', alpha=0.7)

    # Fuerza (repetida para comparar)
    axs_lin[1, 1].plot(x_tiempo, historial_fuerza, color='purple', linewidth=1.5)
    axs_lin[1, 1].set_title("Fuerza Aplicada (N)")
    axs_lin[1, 1].grid(True, linestyle='--', alpha=0.7)

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
simular_animado(15, 0.01, 89, 0, 0)