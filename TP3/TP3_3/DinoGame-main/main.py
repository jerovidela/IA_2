import subprocess
try:
    import pygame
except ImportError as err:
    subprocess.check_call(['pip', 'install', 'pygame'])
    import pygame

import os
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import random
import csv
from datetime import datetime
import numpy as np
from Dinosaur import Dinosaur
from Cloud import Cloud
from Bird import Bird
from SmallCactus import SmallCactus
from LargeCactus import LargeCactus
#from Genetic import updateNetwork
from Genetic import GeneticAlgorithm
from ImageCapture import ImageCapture

screen_spawn_position = (100, 100)
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % screen_spawn_position
pygame.init()

# Global Constants
SCREEN_HEIGHT = 600
SCREEN_WIDTH = 1100
SCREEN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("DinoGame")
generation = 1
bestScore = 0
playMode = "X"
csv_filename = f"evolution_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
imageCapture = ImageCapture(screen_spawn_position)

BG = pygame.image.load(os.path.join("Assets/Other", "Track.png"))

def populate(population_size):
    population = []
    loaded_brains = None
    if os.path.isfile('saved_brains.npz'):
        loaded_brains = np.load('saved_brains.npz')
        print("Cerebros de la generacion previa cargados con éxito.")
    for i in range(population_size):
        while True:
            R = random.randint(0, 255)
            G = random.randint(0, 255)
            B = random.randint(0, 255)
            brightness = 0.299 * R + 0.587 * G + 0.114 * B
            if brightness < 180:  # Evitar colores demasiado claros
                break
        color = (R, G, B)

        new_dino = Dinosaur(i, color, autoplay=True)
        
        # Inyectar genes si existen para toda la matriz poblacional
        if loaded_brains is not None and i < len(loaded_brains['W1']):
            new_dino.W1 = loaded_brains['W1'][i]
            new_dino.b1 = loaded_brains['b1'][i]
            new_dino.W2 = loaded_brains['W2'][i]
            new_dino.b2 = loaded_brains['b2'][i]
            
        population.append(new_dino)
    return population


# ======================== SELECT THE POPULATION NUMBER PLAYING AT THE SAME TIME ======================
population_number = 100
# =====================================================================================================
population = populate(population_number)
player = Dinosaur(0)
callUpdateNetwork = False

def gameScreen():
    global game_speed, x_pos_bg, y_pos_bg, points, obstacles, population, callUpdateNetwork, generation, bestScore, playMode
    run = True
    clock = pygame.time.Clock()
    ga = GeneticAlgorithm(mutation_rate=0.10, elitism_count=5, tournament_size=5)
    game_speed = 20
    cloud = Cloud(SCREEN_WIDTH, game_speed)
    x_pos_bg = 0
    y_pos_bg = 380
    points = 0
    font = pygame.font.Font('freesansbold.ttf', 20)
    obstacles = []
    callUpdateNetwork = True

    def score():
        global points, game_speed
        points += 1
        if points % 100 == 0:
            game_speed += 1

        text = font.render("Puntos: " + str(points), True, (0, 0, 0))
        textRect = text.get_rect()
        textRect.center = (1000, 40)
        SCREEN.blit(text, textRect)

    def countSurviving():
        global population
        text = font.render("Vivos: " + str(count_alive(population)), True, (0, 0, 0))
        textRect = text.get_rect()
        textRect.center = (1000, 65)
        SCREEN.blit(text, textRect)

    def currentGeneration():
        global generation
        text = font.render("Generación: " + str(generation), True, (0, 0, 0))
        textRect = text.get_rect()
        textRect.center = (1000, 90)
        SCREEN.blit(text, textRect)

    def background():
        global x_pos_bg, y_pos_bg
        image_width = BG.get_width()
        SCREEN.blit(BG, (x_pos_bg, y_pos_bg))
        SCREEN.blit(BG, (image_width + x_pos_bg, y_pos_bg))
        if x_pos_bg <= -image_width:
            SCREEN.blit(BG, (image_width + x_pos_bg, y_pos_bg))
            x_pos_bg = 0
        x_pos_bg -= game_speed

    def deathUpdates(player, obstacle):
        #global generation, points, bestScore
        global generation, points, bestScore, population, game_speed, obstacles
        obstacle_params = obstacle.rect
        SCREEN.fill((255, 255, 255))
        background()
        score()
        cloud.draw(SCREEN)
        cloud.update()
        SCREEN.blit(player.image, (player.dino_rect.x, player.dino_rect.y))
        SCREEN.blit(obstacle.image[obstacle.type], (obstacle_params.x, obstacle_params.y))
        pygame.draw.rect(SCREEN, (255, 0, 0), player.dino_rect, 2)
        pygame.draw.rect(SCREEN, (0, 0, 255), obstacle_params, 2)
# ================= VERIFICACIÓN DE EXTINCIÓN =================
        if all(not dino.alive for dino in population) and playMode not in ['m', 'c', 'a', 'e']:
            
            # Cálculo de métricas
            max_score = max(dino.score for dino in population)
            avg_score = sum(dino.score for dino in population) / len(population)
            
            print(f"--- Fin de Generación {generation} ---")
            print(f"Máximo puntaje: {max_score}")
            print(f"Promedio de puntos: {avg_score:.2f}\n")


            file_exists = os.path.isfile(csv_filename)
            with open(csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                if not file_exists:
                    # Si el archivo es nuevo, creamos los encabezados
                    writer.writerow(['Generacion', 'Max_Score', 'Avg_Score'])
                # Guardamos los datos de esta generación
                writer.writerow([generation, max_score, round(avg_score, 2)])
        
            # Ordenar por puntaje y guardar la población completa (los 100 individuos)
            sorted_pop = sorted(population, key=lambda x: x.score, reverse=True)
            
            # Guardar todas las matrices en un único archivo comprimido
            np.savez('saved_brains.npz', 
                     W1=[d.W1 for d in sorted_pop], b1=[d.b1 for d in sorted_pop],
                     W2=[d.W2 for d in sorted_pop], b2=[d.b2 for d in sorted_pop])
            

            if max_score > bestScore:
                bestScore = max_score

            # Evolucionar y resetear
            population = ga.next_generation(population)
            obstacles.clear()
            game_speed = 20
            points = 0
            
        elif playMode in ['m', 'c', 'a', 'e']:
            if points > bestScore:
                bestScore = points
        # =============================================================
        pygame.display.update()
        pygame.time.delay(1000)
        generation += 1
        menu()

    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        SCREEN.fill((255, 255, 255))

        if playMode == 'm' or playMode == 'c':
            userInput = pygame.key.get_pressed()
            player.draw(SCREEN)
            player.update(userInput)
            if playMode == 'c':
                imageCapture.capture(userInput)

        elif playMode == 'a':
            if player.alive:
                player.draw(SCREEN)
                imageCapture.capture_live()
                player.predict()

        elif playMode == 'e':
            if player.alive:
                player.draw(SCREEN)
                closest_obstacle = None
                if len(obstacles) > 0:
                    closest_obstacle = obstacles[0]
                    for obs in obstacles:
                        if obs.rect.x + obs.rect.width > 80:
                            closest_obstacle = obs
                            break
                if closest_obstacle is not None:
                    dist = closest_obstacle.rect.x - player.dino_rect.x
                    inputs = [game_speed, closest_obstacle.rect.y, closest_obstacle.rect.height, closest_obstacle.rect.width, dist, player.dino_rect.y]
                else:
                    inputs = [game_speed, 0, 0, 0, 1000, player.dino_rect.y]
                    
                accion = player.act(inputs)
                player.update(accion)

        else:
            # 1. Identificar el obstáculo más cercano (fuera del bucle de los dinos)
            closest_obstacle = None
            if len(obstacles) > 0:
                closest_obstacle = obstacles[0]
                # Asegurarnos de que el obstáculo no haya quedado ya detrás del dinosaurio
                for obs in obstacles:
                    # Si el borde derecho del obstáculo está por delante de la posición X del dino
                    if obs.rect.x + obs.rect.width > 80:
                        closest_obstacle = obs
                        break

            # 2. Iterar sobre la población de dinosaurios vivos
            for dino in population:
                if dino.alive:
                    dino.draw(SCREEN)
                    
                    # 3. Sensado de variables (con bloque de contingencia)
                    if closest_obstacle is not None:
                        v_game = game_speed
                        y_obst = closest_obstacle.rect.y
                        h_obst = closest_obstacle.rect.height
                        w_obst = closest_obstacle.rect.width
                        dist = closest_obstacle.rect.x - dino.dino_rect.x
                        y_dino = dino.dino_rect.y
                    else:
                        # Valores seguros por defecto si la pantalla está limpia
                        v_game = game_speed
                        y_obst = 0
                        h_obst = 0
                        w_obst = 0
                        dist = 1000  # Distancia artificialmente alta para evitar falsos positivos
                        y_dino = dino.dino_rect.y

                    # 4. Empaquetar las 6 entradas (el orden debe coincidir con NeuralNetwork.py)
                    # 4. Empaquetar las 6 entradas NORMALIZADAS
                    inputs = [
                        v_game / 50.0,                 # Velocidad max aprox
                        y_obst / SCREEN_HEIGHT, 
                        h_obst / 100.0,                # Altura max aprox
                        w_obst / 100.0,                # Ancho max aprox
                        dist / SCREEN_WIDTH,           # Distancia max
                        y_dino / SCREEN_HEIGHT
                    ]
                    
                    # 5. Ejecutar el cerebro y actualizar las físicas UNA sola vez por frame
                    accion = dino.act(inputs)
                    dino.update(accion)


        if len(obstacles) == 0:
            if random.randint(0, 2) == 0:
                obstacles.append(SmallCactus(SCREEN_WIDTH, game_speed, obstacles))
            elif random.randint(0, 2) == 1:
                obstacles.append(LargeCactus(SCREEN_WIDTH, game_speed, obstacles))
            elif random.randint(0, 2) == 2:
                obstacles.append(Bird(SCREEN_WIDTH, game_speed, obstacles))
        

        for obstacle in obstacles:
            obstacle.draw(SCREEN)
            obstacle.update()
            obstacle_params = obstacle.rect

            if playMode == 'm' or playMode == 'c' or playMode == 'a' or playMode == 'e':
                if player.dino_rect.colliderect(obstacle_params):
                    player.alive = False
                    
            else:
                for dino in population:
                    dino_params = dino.dino_rect
                    if dino.alive and dino_params.colliderect(obstacle_params):
                        dino.score = points
                        dino.alive = False

                        if (count_alive(population) == 0):
                            last_dino = dino

        if ((playMode == 'm' or playMode == 'c' or playMode == 'a' or playMode == 'e') and player.alive == False):
            deathUpdates(player, obstacle)
        elif (playMode != 'm' and playMode != 'c' and playMode != 'a' and playMode != 'e' and count_alive(population) == 0):
            countSurviving()
            currentGeneration()
            deathUpdates(last_dino, obstacle)

        background()

        cloud.draw(SCREEN)
        cloud.update()

        score()

        if (playMode != 'm' and playMode != 'c' and playMode != 'a'):
            countSurviving()
            currentGeneration()

        clock.tick(30)
        pygame.display.update()

def menu():
    global callUpdateNetwork, generation, bestScore, playMode, population, population_number
    run = True
    warning_text = False
    if playMode in ['m', 'c', 'a', 'e']:
        player.resetStatus()

    elif playMode not in ['m', 'c', 'a', 'e'] and callUpdateNetwork:
        callUpdateNetwork = False
        for dino in population:
            dino.resetStatus()
        
    while run:
        SCREEN.fill((255, 255, 255))
        font = pygame.font.Font('freesansbold.ttf', 30)

        if generation == 1:
            text = font.render("Pulse 'm' para jugar manualmente", True, (0, 0, 0))

            auxText = font.render("'c' para capturar imágenes", True, (0, 0, 0))
            auxTextRect = auxText.get_rect()
            auxTextRect.center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 50)
            SCREEN.blit(auxText, auxTextRect)

            auxText = font.render("'a' para usar el modelo generado por Tensorflow", True, (0, 0, 0))
            auxTextRect = auxText.get_rect()
            auxTextRect.center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 100)
            SCREEN.blit(auxText, auxTextRect)

            auxText = font.render("o cualquier otra letra para jugar automáticamente", True, (0, 0, 0))
            auxTextRect = auxText.get_rect()
            auxTextRect.center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 150)
            SCREEN.blit(auxText, auxTextRect)

            auxText = font.render("'e' para modo Exhibición (Mejor Dino)", True, (0, 0, 0))
            auxTextRect = auxText.get_rect()
            auxTextRect.center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 200)
            SCREEN.blit(auxText, auxTextRect)
            
            if warning_text:
                warn_font = pygame.font.Font('freesansbold.ttf', 40)
                warn = warn_font.render("Algoritmo no entrenado", True, (255, 0, 0))
                warnRect = warn.get_rect()
                warnRect.center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 260)
                SCREEN.blit(warn, warnRect)
        elif generation > 1:
            text = font.render("Pulse cualquier tecla para reiniciar", True, (0, 0, 0))
            score = font.render("Mejor puntuación: " + str(bestScore), True, (0, 0, 0))
            scoreRect = score.get_rect()
            scoreRect.center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 50)
            SCREEN.blit(score, scoreRect)


        textRect = text.get_rect()

        textRect.center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
        SCREEN.blit(text, textRect)
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
                if generation == 1:
                    key_pressed = pygame.key.name(event.key)
                    if key_pressed == 'e':
                        if os.path.isfile('saved_brains.npz'):
                            loaded_brains = np.load('saved_brains.npz')
                            player.W1 = loaded_brains['W1'][0]
                            player.b1 = loaded_brains['b1'][0]
                            player.W2 = loaded_brains['W2'][0]
                            player.b2 = loaded_brains['b2'][0]
                            playMode = 'e'
                            population = []
                            gameScreen()
                        else:
                            warning_text = True
                    else:
                        playMode = key_pressed
                        if playMode in ['m', 'c', 'a']:
                            population = []
                        gameScreen()

def count_alive(population):
    alive = 0
    for dino in population:
        if dino.alive:
            alive += 1
    return alive

if __name__ == "__main__":
    menu()
