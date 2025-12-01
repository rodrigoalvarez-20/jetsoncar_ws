
"""    tsp_navigation.py
    Lectura de cámara por URL (OpenCV) + detección de cilindros rojos + Algoritmo Genético (TSP)
    Autor: Shareni y ro             """

import time
import math
import random
from copy import deepcopy

import numpy as np
import cv2
import matplotlib.pyplot as plt

import pyrealsense2 as rs
import serial
import onnxruntime as ort
from threading import Thread, Lock, Event


# ===================== CONFIGURACIÓN =====================

# Área y grid
AREA_M = 5.0          # 5 x 5 metros
GRID_SIZE = 100       # 100x100 celdas => r = 0.05 m/celda
R = AREA_M / GRID_SIZE

# Detección rojo (HSV)
HSV_ROJO_1_LOW  = np.array([0, 120, 70])
HSV_ROJO_1_HIGH = np.array([10, 255, 255])
HSV_ROJO_2_LOW  = np.array([170, 120, 70])
HSV_ROJO_2_HIGH = np.array([180, 255, 255])

AREA_MIN_PX = 600  # filtrar pequeños ruidos (ajusta según cámara/distancia)

# AG (parámetros)
POP_SIZE = 80
GENERATIONS = 300
CROSSOVER_RATE = 0.9
MUTATION_RATE = 0.12
ELITISM = 2

# Serial Bluetooth (por defecto rfcomm)
SERIAL_PORT = "/dev/rfcomm0"   # CAMBIA si tu puerto es otro (ej. /dev/ttyACM0)
SERIAL_BAUD = 115200

# Tiempo entre comandos físicos para dar tiempo al Arduino (segundos)
DELAY_COMANDOS = 1.2

# Bandera modo cámara
USE_CAMERA = True

_fig = None
_ax = None




# Modelo YOLO (detección de personas)
YOLO_MODEL_PATH = "models/yolo11n_320_half_nms_cuda_21.onnx"  # ruta al modelo
YOLO_CLASSES = {0: "person"}  
YOLO_SCORE_THRESHOLD = 0.6 

# Inicializar modelo YOLO
yolo_session = ort.InferenceSession(
    YOLO_MODEL_PATH,
    providers=["CUDAExecutionProvider", "CPUExecutionProvider"]
)
input_name = yolo_session.get_inputs()[0].name
input_h, input_w = yolo_session.get_inputs()[0].shape[2:]



# ===================== UTILIDADES =====================

def meters_to_cell(x_m, y_m):
    i = int(np.clip(x_m / R, 0, GRID_SIZE - 1))
    j = int(np.clip(y_m / R, 0, GRID_SIZE - 1))
    return i, j

def cell_to_meters(i, j):
    x = (i + 0.5) * R
    y = (j + 0.5) * R
    return x, y

def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])



# ===================== Occupancy Grid =====================

class OccupancyGrid:
    def __init__(self, size=GRID_SIZE):
        self.size = size
        self.grid = np.zeros((size, size), dtype=np.uint8)  # 0 libre,1 obst,2 nodo
    def clear(self):
        self.grid.fill(0)
    def set_node(self, i, j):
        self.grid[j, i] = 2
    def set_obstacle(self, i, j):
        self.grid[j, i] = 1
    def get_nodes_as_meters(self):
        nodes = []
        for j in range(self.size):
            for i in range(self.size):
                if self.grid[j, i] == 2:
                    nodes.append(cell_to_meters(i, j))
        return nodes

# ===================== RealSense (SR300) =====================

def init_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    # color stream 640x480@30
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    print("[SR300] Cámara iniciada.")
    return pipeline, profile

"""# ===================== Detección de nodos rojos =====================

def detectar_nodos_rojos(frame, area_min_px=AREA_MIN_PX):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, HSV_ROJO_1_LOW, HSV_ROJO_1_HIGH)
    mask2 = cv2.inRange(hsv, HSV_ROJO_2_LOW, HSV_ROJO_2_HIGH)
    mask = cv2.bitwise_or(mask1, mask2)
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detecciones = []
    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area < area_min_px:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cx = x + w//2
        cy = y + h//2
        detecciones.append((cx, cy, area, (x,y,w,h)))
    return mask, detecciones"""

# Convertir pixel -> metros (asume la cámara cubre exactamente el área AREA_M)
def pixel_to_meters(cx, cy, frame_w, frame_h):
    x_m = (cx / frame_w) * AREA_M
    y_m = (cy / frame_h) * AREA_M
    return x_m, y_m

def actualizar_grid_con_detecciones(grid: OccupancyGrid, detecciones_px, frame_w, frame_h):
    # limpiar nodos previos
    for j in range(grid.size):
        for i in range(grid.size):
            if grid.grid[j,i] == 2:
                grid.grid[j,i] = 0
    # agregar nuevas detecciones
    for (cx, cy, area, bbox) in detecciones_px:
        x_m, y_m = pixel_to_meters(cx, cy, frame_w, frame_h)
        i, j = meters_to_cell(x_m, y_m)
        grid.set_node(i, j)
      
def detectar_personas(frame, yolo_session, input_name, input_w, input_h):
        detecciones = []
        h, w = frame.shape[:2]
        frame_resized = cv2.resize(frame, (input_w, input_h))
        frame_input = frame_resized.astype(np.float32) / 255.0
        frame_input = np.expand_dims(np.transpose(frame_input, (2, 0, 1)), axis=0)

        model_out = yolo_session.run(None, {input_name: frame_input})[0][0]
        #print(model_out)
        for det in model_out:
            x1, y1, x2, y2, score, cls_id = det
            if score < YOLO_SCORE_THRESHOLD:
                continue
            cls_id = int(cls_id)
            if YOLO_CLASSES.get(cls_id) != "person":
                continue

            # Escalar coordenadas
            x1 = int(x1 * w / input_w)
            x2 = int(x2 * w / input_w)
            y1 = int(y1 * h / input_h)
            y2 = int(y2 * h / input_h)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            detecciones.append((cx, cy, score, (x1, y1, x2 - x1, y2 - y1)))

        return detecciones
        



# ===================== Algoritmo Genético (TSP) =====================

def create_distance_matrix(nodes):
    n = len(nodes)
    D = [[0.0]*n for _ in range(n)]
    for a in range(n):
        for b in range(n):
            D[a][b] = euclidean(nodes[a], nodes[b])
    return D

def total_distance(route, D):
    dist = 0.0
    n = len(route)
    for k in range(n-1):
        dist += D[route[k]][route[k+1]]
    dist += D[route[-1]][route[0]]
    return dist

def random_route(n):
    route = list(range(n))
    random.shuffle(route)
    return route

def initial_population(n, pop_size):
    return [random_route(n) for _ in range(pop_size)]

def tournament_selection(pop, fitnesses, k=3):
    aspirantes = random.sample(range(len(pop)), k)
    mejor = min(aspirantes, key=lambda idx: fitnesses[idx])
    return pop[mejor]

def order_crossover(p1, p2):
    n = len(p1)
    a, b = sorted(random.sample(range(n), 2))
    child = [-1]*n
    child[a:b+1] = p1[a:b+1]
    pos = (b+1)%n
    idx = (b+1)%n
    while -1 in child:
        gene = p2[idx]
        if gene not in child:
            child[pos] = gene
            pos = (pos+1)%n
        idx = (idx+1)%n
    return child

def swap_mutation(route):
    a, b = random.sample(range(len(route)), 2)
    route[a], route[b] = route[b], route[a]

def evolve_population(pop, D):
    pop_size = len(pop)
    fitnesses = [total_distance(ind, D) for ind in pop]
    new_pop = []
    sorted_idx = sorted(range(pop_size), key=lambda i: fitnesses[i])
    for e in range(min(ELITISM, pop_size)):
        new_pop.append(deepcopy(pop[sorted_idx[e]]))
    while len(new_pop) < pop_size:
        parent1 = tournament_selection(pop, fitnesses, k=3)
        parent2 = tournament_selection(pop, fitnesses, k=3)
        if random.random() < CROSSOVER_RATE:
            child1 = order_crossover(parent1, parent2)
            child2 = order_crossover(parent2, parent1)
        else:
            child1, child2 = deepcopy(parent1), deepcopy(parent2)
        if random.random() < MUTATION_RATE:
            swap_mutation(child1)
        if random.random() < MUTATION_RATE:
            swap_mutation(child2)
        new_pop.append(child1)
        if len(new_pop) < pop_size:
            new_pop.append(child2)
    return new_pop

def run_genetic_tsp(nodes, pop_size=POP_SIZE, generations=GENERATIONS, seed_route=None, verbose=False):
    n = len(nodes)
    if n < 2:
        return None, None
    D = create_distance_matrix(nodes)
    if seed_route is not None and len(seed_route) == n:
        pop = [deepcopy(seed_route)]
        while len(pop) < pop_size:
            pop.append(random_route(n))
    else:
        pop = initial_population(n, pop_size)
    best = None
    best_dist = float('inf')
    for g in range(generations):
        for ind in pop:
            d = total_distance(ind, D)
            if d < best_dist:
                best_dist = d
                best = deepcopy(ind)
        pop = evolve_population(pop, D)
        if verbose and (g % max(1, generations//10) == 0):
            print(f"Gen {g}: best {best_dist:.3f}")
    return best, best_dist
 

# ===================== Visualización =====================

def dibujar_ruta(grid: OccupancyGrid, nodes_m, ruta_indices, titulo="Ruta TSP"):
    global _fig, _ax

    # Activate interactive mode
    plt.ion()

    # Create figure only once
    if _fig is None or _ax is None:
        _fig, _ax = plt.subplots(figsize=(6, 6))
        _fig.canvas.manager.set_window_title(titulo)

    # Clear previous frame
    _ax.clear()

    # Build image
    img = np.ones((grid.size, grid.size, 3), dtype=np.uint8) * 255
    for j in range(grid.size):
        for i in range(grid.size):
            val = grid.grid[j, i]
            if val == 1:
                img[j, i] = np.array([150, 150, 150])
            elif val == 2:
                img[j, i] = np.array([200, 60, 60])

    scale = 4
    img_up = cv2.resize(img, (grid.size * scale, grid.size * scale), interpolation=cv2.INTER_NEAREST)
    _ax.imshow(img_up)

    # Plot nodes
    if nodes_m:
        for (x, y) in nodes_m:
            i, j = meters_to_cell(x, y)
            _ax.plot((i + 0.5) * scale, (j + 0.5) * scale, 'ro')

    # Plot route
    if ruta_indices:
        pts = []
        for idx in ruta_indices:
            x, y = nodes_m[idx]
            i, j = meters_to_cell(x, y)
            pts.append(((i + 0.5) * scale, (j + 0.5) * scale))
        if len(pts) > 0:
            pts.append(pts[0])
            pts = np.array(pts)
            _ax.plot(pts[:, 0], pts[:, 1], '-b')

    _ax.set_title("Prueba")
    _ax.axis('off')

    # Update existing figure without opening new ones
    _fig.canvas.draw()
    _fig.canvas.flush_events()
    plt.pause(0.05)

# ===================== Serial Bluetooth =====================

def conectar_serial(port=SERIAL_PORT, baud=SERIAL_BAUD, timeout=1):
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)
        print(f"[Serial] Conectado a {port} @ {baud}")
        return ser
    except Exception as e:
        print("[Serial] Error al conectar:", e)
        return None

def enviar_comando_ser(ser, cmd):
    if ser and ser.is_open:
        try:
            ser.write((cmd + "\n").encode('utf-8'))
            print("[Serial] Enviado:", cmd)
        except Exception as e:
            print("[Serial] Error enviando:", e)
    else:
        print("[SIM] enviar:", cmd)

# ===================== BUCLE PRINCIPAL =====================

def main():
    grid = OccupancyGrid(GRID_SIZE)
    #pipeline = None
    #serial_conn = None
    camera = None
    if USE_CAMERA:
        try:
            #pipeline, profile = init_realsense()
            camera = cv2.VideoCapture(0)
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not camera.isOpened():
                raise Exception("No se pudo abrir la cámara.")
        except Exception as e:
            print("[SR300] No se pudo iniciar la cámara:", e)
            USE_CAM = False
            pipeline = None

    # intentar conectar serial (bluetooth)
    #serial_conn = conectar_serial(SERIAL_PORT, SERIAL_BAUD)

    previous_route = None
    last_nodes_m = []
    
    frame_counter = 0
    DETECTION_INTERVAL = 10
    last_detecciones = []
    last_frame_ruta = None

    try:
        while True:
            if USE_CAMERA and camera is not None:
                #frames = pipeline.wait_for_frames()
                #color_frame = frames.get_color_frame()
                ret, frame = camera.read()
                if not ret:
                    continue
                #frame = np.asanyarray(color_frame.get_data())
                h, w = frame.shape[:2]
                frame_counter += 1

                """mask, detecciones = detectar_nodos_rojos(frame)"""
                
                if frame_counter % DETECTION_INTERVAL == 0:
                    
                    #Detectar personas
                    detecciones = detectar_personas(frame, yolo_session, input_name, 320, 320)
                    print(f"Detecciones personas: {len(detecciones)}")
                    
                    last_detecciones = detecciones 

                    actualizar_grid_con_detecciones(grid, detecciones, w, h)
                    nodes_m = grid.get_nodes_as_meters()
                    
                    # dibujar detecciones sobre el frame actual
                    for (cx, cy, score, bbox) in detecciones:
                        print(f" - Persona score={score:.2f} at ({cx},{cy})")
                        x,y,wb,hb = bbox
                        cv2.rectangle(frame, (x,y), (x+wb, y+hb), (0,255,0), 2)
                        cv2.circle(frame, (cx,cy), 4, (255,255,255), -1)

                    # Calcular ruta TSP
                    if len(nodes_m) >= 2:
                        ruta, dist = run_genetic_tsp(nodes_m, seed_route=previous_route, verbose=False)
                        if ruta is not None:
                            
                            print(f"Ruta actualizada, dist: {dist:.3f} m")
                            
                            #dibujar_ruta(grid, nodes_m, ruta, titulo=f"Ruta TSP dist={dist:.2f} m")
                            
                            D = create_distance_matrix(nodes_m)

                            print(f"Ruta indices: {ruta} dist: {dist:.3f} m")

                            n = len(ruta)
                            for i in range(n):
                                nodo_a_idx = ruta[i]
                                nodo_b_idx = ruta[(i + 1) % n] # El % n maneja el cierre del ciclo
                                dist_segmento = D[nodo_a_idx][nodo_b_idx]
                                print(f"  {nodo_a_idx} -> {nodo_b_idx}: {dist_segmento:.3f} m")
                            
                            
                            previous_route = ruta
                            last_nodes_m = nodes_m                         
                            
                            
                            # Dibujar la ruta sobre el frame
                            if len(detecciones) == len(nodes_m):
                                n_ruta = len(ruta)
                                for i in range(n_ruta):
                                    idx_actual = ruta[i]
                                    idx_siguiente = ruta[(i + 1) % n_ruta]
                                    (cx_actual, cy_actual, _, _) = detecciones[idx_actual]
                                    (cx_siguiente, cy_siguiente, _, _) = detecciones[idx_siguiente]
                                    cv2.line(frame, (cx_actual, cy_actual), (cx_siguiente, cy_siguiente), (255, 0, 0), 2)
                                    if i == 0:
                                        cv2.circle(frame, (cx_actual, cy_actual), 10, (0, 255, 255), 2)

                    last_frame_ruta = frame.copy()
                else:
                    if last_frame_ruta is not None:
                        for (cx, cy, score, bbox) in last_detecciones:
                            x,y,wb,hb = bbox
                            cv2.rectangle(frame, (x,y), (x+wb, y+hb), (0,255,0), 2)
                        
                        # Dibujar ruta antigua
                        if previous_route and len(last_detecciones) == len(last_nodes_m):
                             n_ruta = len(previous_route)
                             for i in range(n_ruta):
                                idx_actual = previous_route[i]
                                idx_siguiente = previous_route[(i + 1) % n_ruta]
                                (cx_actual, cy_actual, _, _) = last_detecciones[idx_actual]
                                (cx_siguiente, cy_siguiente, _, _) = last_detecciones[idx_siguiente]
                                cv2.line(frame, (cx_actual, cy_actual), (cx_siguiente, cy_siguiente), (255, 0, 0), 2)

                # Mostrar el frame
                
                #mostrar u ocultar camara bro (el cv2)
                #cv2.imshow("Camara", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Salida por 'q'")
                    break
                if key == ord('s'):
                    # iniciar ejecución física: enviar cada nodo en orden
                    if previous_route and last_nodes_m:
                        print("[EJECUCIÓN] Enviando ruta al carrito...")
                        for idx in previous_route:
                            x_t, y_t = last_nodes_m[idx]
                            cmd = f"MOVE {x_t:.3f},{y_t:.3f}"
                            #enviar_comando_ser(serial_conn, cmd)
                            time.sleep(DELAY_COMANDOS)
                        # regresar al inicio
                        start_idx = previous_route[0]
                        x0,y0 = last_nodes_m[start_idx]
                        #enviar_comando_ser(serial_conn, f"MOVE {x0:.3f},{y0:.3f}")
                        print("[EJECUCIÓN] Ruta enviada.")
                    else:
                        print("[EJECUCIÓN] No hay ruta calculada para enviar.")
            else:
                # MODO SIMULACIÓN corto: crear nodos y correr AG una vez
                nodes_m = [(0.5,0.5),(4.2,0.7),(3.5,3.8),(0.6,4.6)]
                grid.clear()
                for (x,y) in nodes_m:
                    i,j = meters_to_cell(x,y)
                    grid.set_node(i,j)
                ruta, dist = run_genetic_tsp(nodes_m, verbose=True)
                print("Simulado ruta:", ruta, "dist:", dist)
                dibujar_ruta(grid, nodes_m, ruta, titulo=f"Simulación ruta dist={dist:.2f}")
                break

    except KeyboardInterrupt:
        print("Interrumpido por usuario (CTRL+C).")
    finally:
        #if pipeline is not None:
        #    pipeline.stop()
        cv2.destroyAllWindows()
        #if serial_conn and serial_conn.is_open:
        #    serial_conn.close()
        print("Programa finalizado.")

if __name__ == "__main__":
    main()
