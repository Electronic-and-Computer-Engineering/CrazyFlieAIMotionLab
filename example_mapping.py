import time
import math
import os
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import convolve, binary_dilation, binary_erosion, median_filter
from scipy.ndimage import gaussian_filter
from skimage.morphology import skeletonize
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.platformservice import PlatformService
from cflib.crazyflie.log import LogConfig


URI = "radio://0/33/2M/E7E7E7E7AA"
HEIGHT = 0.5
MOVE_SPEED = 0.5
GRID_RES = 0.05

# Globale Variablen
grid_map = {}
path_x = []
path_y = []
start_offset = {'x': None, 'y': None}
current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'vx': 0.0, 'vy': 0.0}
current_ranges = {'front': 2000, 'back': 2000, 'left': 2000, 'right': 2000, 'up': 2000}
flow_quality = 0
is_flying = False
low_battery = False

# Drift-Kompensation
drift_estimator = {'x': 0.0, 'y': 0.0, 'samples': 0}

# Plot Setup
fig, ax = plt.subplots(figsize=(8, 8))
scat_map = ax.scatter([], [], c=[], cmap='inferno', s=15, marker='s', alpha=0.8, label='Wände')
cbar = fig.colorbar(scat_map, ax=ax)
cbar.set_label('Anzahl Messungen (Dichte)')
line_path, = ax.plot([], [], 'r-', linewidth=1, alpha=0.8, label='Pfad')
quiver_drone = ax.quiver(0, 0, 1, 0, color='red', scale=20, width=0.01, headwidth=3, zorder=10)


def setup_plot():
    plt.ion()
    ax.set_aspect('equal', 'box')
    ax.set_xlim(-2.0, 2.0)
    ax.set_ylim(-2.0, 2.0)
    ax.grid(True)
    ax.legend()
    plt.title("Exploration Mission")
    plt.show()


def update_plot():
    if not grid_map: return

    current_grid = grid_map.copy()
    if not current_grid: return

    yaw_rad = math.radians(current_pose['yaw'])
    quiver_drone.set_offsets([[current_pose['x'], current_pose['y']]])
    quiver_drone.set_UVC(math.cos(yaw_rad), math.sin(yaw_rad))

    line_path.set_xdata(path_x)
    line_path.set_ydata(path_y)

    keys = list(current_grid.keys())
    vals = list(current_grid.values())

    if len(keys) > 0:
        points = np.array(keys) * GRID_RES
        vals_array = np.array(vals)

        scat_map.set_offsets(points)
        scat_map.set_array(vals_array)

        vmax = np.percentile(vals_array, 95) if len(vals_array) > 0 else 10
        scat_map.set_clim(vmin=1, vmax=vmax)

        xlim = ax.get_xlim()
        ylim = ax.get_ylim()

        path_min_x = min(path_x) if path_x else 0
        path_max_x = max(path_x) if path_x else 0
        path_min_y = min(path_y) if path_y else 0
        path_max_y = max(path_y) if path_y else 0

        points_min_x = np.min(points[:, 0])
        points_max_x = np.max(points[:, 0])
        points_min_y = np.min(points[:, 1])
        points_max_y = np.max(points[:, 1])

        data_min_x = min(points_min_x, path_min_x)
        data_max_x = max(points_max_x, path_max_x)
        data_min_y = min(points_min_y, path_min_y)
        data_max_y = max(points_max_y, path_max_y)

        margin = 0.5
        if data_min_x < xlim[0]: ax.set_xlim(left=data_min_x - margin)
        if data_max_x > xlim[1]: ax.set_xlim(right=data_max_x + margin)
        if data_min_y < ylim[0]: ax.set_ylim(bottom=data_min_y - margin)
        if data_max_y > ylim[1]: ax.set_ylim(top=data_max_y + margin)

    fig.canvas.draw_idle()
    plt.pause(0.001)


def log_callback(timestamp, data, logconf):
    global flow_quality

    # Position mit Flow Deck Unterstützung
    if 'stateEstimate.x' in data:
        raw_x = data['stateEstimate.x']
        raw_y = data['stateEstimate.y']
        yaw_deg = data['stateEstimate.yaw']

        if start_offset['x'] is None:
            start_offset['x'] = raw_x
            start_offset['y'] = raw_y

        pos_x = raw_x - start_offset['x']
        pos_y = raw_y - start_offset['y']

        current_pose['x'] = pos_x
        current_pose['y'] = pos_y
        current_pose['yaw'] = yaw_deg

        if is_flying:
            path_x.append(pos_x)
            path_y.append(pos_y)

    # Geschwindigkeit
    if 'stateEstimate.vx' in data:
        current_pose['vx'] = data['stateEstimate.vx']
        current_pose['vy'] = data['stateEstimate.vy']

    # Flow Deck Quality (falls vorhanden)
    if 'motion.deltaX' in data:
        # Flow Deck ist aktiv
        flow_quality = 1

    # Ranges - ALLE Sensoren
    if 'range.front' in data:
        current_ranges['front'] = data['range.front']
        current_ranges['left'] = data['range.left']
        current_ranges['back'] = data['range.back']
        current_ranges['right'] = data['range.right']
        if 'range.up' in data:
            current_ranges['up'] = data['range.up']

    if not is_flying:
        return

    # Wände berechnen - ALLE 4 Richtungen nutzen
    yaw_rad = math.radians(current_pose['yaw'])
    sensors = [
        (current_ranges['front'], 0), 
        (current_ranges['left'], 90),
        (current_ranges['back'], 180), 
        (current_ranges['right'], -90)
    ]

    for dist_mm, angle_offset in sensors:
        dist_m = dist_mm / 1000.0
        if 0.02 < dist_m < 3.0:
            total_angle = yaw_rad + math.radians(angle_offset)
            wall_x = current_pose['x'] + dist_m * math.cos(total_angle)
            wall_y = current_pose['y'] + dist_m * math.sin(total_angle)

            ix = int(round(wall_x / GRID_RES))
            iy = int(round(wall_y / GRID_RES))

            grid_map[(ix, iy)] = grid_map.get((ix, iy), 0) + 1


def bat_log_callback(timestamp, data, logconf):
    global low_battery
    if 'pm.vbat' in data:
        vbat = data['pm.vbat']
        # 3.1V ist kritisch unter Last
        if vbat < 3.1 and not low_battery:
            print(f"\nWARNUNG: Batterie kritisch ({vbat:.2f}V)! Landung eingeleitet.")
            low_battery = True

def get_avoidance_vector():
    """Verbesserte Hinderniserkennung mit allen Sensoren"""
    vx, vy = 0.0, 0.0
    threshold_mm = 600
    avoid_speed = 1.0

    # Gewichtete Repulsion basierend auf Distanz
    def repulsion_force(dist_mm, threshold):
        if dist_mm < threshold:
            return avoid_speed * ((threshold - dist_mm) / threshold) ** 2
        return 0.0

    # Front - zurück und zur Seite
    if current_ranges['front'] < threshold_mm:
        rep = repulsion_force(current_ranges['front'], threshold_mm)
        vx -= rep
        if current_ranges['left'] > current_ranges['right']:
            vy += rep * 0.8
        else:
            vy -= rep * 0.8

    # Back - nach vorne
    if current_ranges['back'] < threshold_mm:
        vx += repulsion_force(current_ranges['back'], threshold_mm)

    # Left - nach rechts
    if current_ranges['left'] < threshold_mm:
        vy -= repulsion_force(current_ranges['left'], threshold_mm)

    # Right - nach links
    if current_ranges['right'] < threshold_mm:
        vy += repulsion_force(current_ranges['right'], threshold_mm)

    return vx, vy


def check_and_correct_drift(scf):
    """Drift-Erkennung mit Threshold"""
    global drift_estimator

    # Drift über Zeit akkumulieren
    drift_estimator['x'] += current_pose['vx'] * 0.1
    drift_estimator['y'] += current_pose['vy'] * 0.1
    drift_estimator['samples'] += 1

    # Alle 5 Sekunden prüfen
    if drift_estimator['samples'] > 50:
        avg_drift = math.sqrt(drift_estimator['x']**2 + drift_estimator['y']**2) / drift_estimator['samples']

        if avg_drift > 0.05:  # 5cm/s Drift
            print(f"WARNUNG: Drift erkannt: {avg_drift:.3f} m/s - Reset Kalman")
            scf.cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            scf.cf.param.set_value('kalman.resetEstimation', '0')
            time.sleep(0.5)

        # Reset Estimator
        drift_estimator = {'x': 0.0, 'y': 0.0, 'samples': 0}


def process_final_map():
    """Berechnet die Wände mittels 2D-Average-Filter"""
    print("\n--- Post-Processing der Karte ---")
    if not grid_map: return

    # 1. Dictionary in Matrix konvertieren
    keys = np.array(list(grid_map.keys()))
    vals = np.array(list(grid_map.values()))
    
    min_x, min_y = np.min(keys, axis=0)
    max_x, max_y = np.max(keys, axis=0)
    
    # Dimensionen berechnen (mit Puffer)
    shape_x = max_x - min_x + 1
    shape_y = max_y - min_y + 1
    grid_matrix = np.zeros((shape_x, shape_y))
    
    # Matrix befüllen
    for (ix, iy), count in grid_map.items():
        grid_matrix[ix - min_x, iy - min_y] = count

    # --- Timestamp für Speicherung ---
    if not os.path.exists('images'):
        os.makedirs('images')
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # --- A. Raw Map speichern (Ungefiltert) ---
    plt.figure("Raw Map", figsize=(8, 8))
    raw_points = keys * GRID_RES
    plt.scatter(raw_points[:, 0], raw_points[:, 1], c=vals, cmap='inferno', s=15, marker='s', alpha=0.8, label='Raw Wände')
    plt.colorbar(label='Dichte (Raw)')
    plt.plot(path_x, path_y, 'r-', linewidth=1, alpha=0.8, label='Pfad')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("Rohdaten Karte")
    filename_raw = os.path.join('images', f"map_raw_{timestamp}.png")
    plt.savefig(filename_raw)
    print(f"Raw Karte gespeichert unter: {filename_raw}")

    # --- B. Filterung (Gefiltert) ---
    # 2. Filterung: Nur sehr leichtes Gaussian Smoothing
    smoothed_matrix = gaussian_filter(grid_matrix, sigma=0.2)

    # 3. Schwellenwert anwenden (Reduziert auf 5%)
    threshold = np.max(smoothed_matrix) * 0.05
    
    clean_keys = []
    clean_vals = []
    
    # Ergebnis zurück in Koordinaten wandeln
    indices = np.argwhere(smoothed_matrix > threshold)
    for ix, iy in indices:
        clean_keys.append((ix + min_x, iy + min_y))
        clean_vals.append(smoothed_matrix[ix, iy])

    print(f"Faltung abgeschlossen. {len(clean_keys)} markante Punkte identifiziert.")

    # 4. Plot aktualisieren
    if clean_keys:
        points = np.array(clean_keys) * GRID_RES
        
        # Neues Fenster für die finale Karte erstellen
        plt.figure("Finale Karte", figsize=(8, 8))
        
        # Scatter Plot (Wände)
        plt.scatter(points[:, 0], points[:, 1], c=clean_vals, cmap='inferno', s=15, marker='s', alpha=0.8, label='Wände')
        plt.colorbar(label='Dichte')
        
        # Pfad einzeichnen (wie im Live-Bild)
        plt.plot(path_x, path_y, 'r-', linewidth=1, alpha=0.8, label='Pfad')
        
        # Limits und Aspect Ratio vom Live-Plot übernehmen
        plt.axis('equal')
        plt.xlim(ax.get_xlim())
        plt.ylim(ax.get_ylim())
        
        plt.grid(True)
        plt.legend()
        plt.title("Finale Karte (Gefiltert)")
        plt.draw()
        plt.pause(0.1) # Sicherstellen, dass das Fenster gerendert wird
        
        # Speichern im images Ordner mit Timestamp (gleicher Timestamp wie Raw)
        filename = os.path.join('images', f"map_filtered_{timestamp}.png")
        plt.savefig(filename)
        print(f"Gefilterte Karte gespeichert unter: {filename}")


def run_rectangle_mission():
    global is_flying
    global low_battery
    cflib.crtp.init_drivers()
    setup_plot()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:

        # 1. Arming
        print("Scharfschalten...")
        ps = PlatformService(scf.cf)
        ps.send_arming_request(True)
        time.sleep(1.0)

        # 2. Flow Deck aktivieren (falls vorhanden)
        try:
            scf.cf.param.set_value('deck.bcFlow2', '1')
            print("Flow Deck aktiviert")
        except:
            print("WARNUNG: Flow Deck nicht gefunden - nutze Standard-Kalman")

        # 3. Kalman Reset
        print("Kalman Reset...")
        time.sleep(2.0)
        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2.0)

        # 4. Logging Setup
        logconf1 = LogConfig(name='State', period_in_ms=10)
        logconf1.add_variable('stateEstimate.x', 'float')
        logconf1.add_variable('stateEstimate.y', 'float')
        logconf1.add_variable('stateEstimate.yaw', 'float')
        logconf1.add_variable('stateEstimate.vx', 'float')
        logconf1.add_variable('stateEstimate.vy', 'float')

        logconf2 = LogConfig(name='Ranges', period_in_ms=10)
        logconf2.add_variable('range.front', 'uint16_t')
        logconf2.add_variable('range.back', 'uint16_t')
        logconf2.add_variable('range.left', 'uint16_t')
        logconf2.add_variable('range.right', 'uint16_t')
        logconf2.add_variable('range.up', 'uint16_t')

        # Batterie Logging (1Hz reicht)
        logconf_bat = LogConfig(name='Battery', period_in_ms=1000)
        logconf_bat.add_variable('pm.vbat', 'float')
        scf.cf.log.add_config(logconf_bat)
        logconf_bat.data_received_cb.add_callback(bat_log_callback)

        # Optional: Flow Deck Logging
        try:
            logconf3 = LogConfig(name='Flow', period_in_ms=50)
            logconf3.add_variable('motion.deltaX', 'int16_t')
            logconf3.add_variable('motion.deltaY', 'int16_t')
            scf.cf.log.add_config(logconf3)
            logconf3.data_received_cb.add_callback(log_callback)
            logconf3.start()
            print("Flow Deck Logging aktiv")
        except:
            pass

        scf.cf.log.add_config(logconf1)
        scf.cf.log.add_config(logconf2)
        logconf1.data_received_cb.add_callback(log_callback)
        logconf2.data_received_cb.add_callback(log_callback)
        logconf_bat.start()
        logconf1.start()
        logconf2.start()

        try:
            # 5. Sanfter Takeoff
            print("Abheben...")
            steps_ramp = 30
            for i in range(steps_ramp):
                h = (i / steps_ramp) * HEIGHT
                scf.cf.commander.send_hover_setpoint(0, 0, 0, h)
                time.sleep(0.1)

            # Stabilisierung
            print("Stabilisiere Position...")
            for _ in range(30):
                scf.cf.commander.send_hover_setpoint(0, 0, 0, HEIGHT)
                v_total = math.sqrt(current_pose['vx']**2 + current_pose['vy']**2)
                if v_total < 0.05:
                    break
                time.sleep(0.1)

            # Nullpunkt setzen
            print("Setze Nullpunkt...")
            start_offset['x'] = None
            start_offset['y'] = None
            time.sleep(1.0)

            is_flying = True
            print("Starte Exploration (Strg+C zum Abbrechen)...")

            # Exploration Parameter
            fw_speed = 0.3
            rot_speed = 15.0
            min_dist_mm = 500
            avoiding_obstacle = False

            loop_count = 0
            last_drift_check = 0

            while True:
                loop_count += 1

                # Sicherheitscheck Batterie
                if low_battery:
                    raise KeyboardInterrupt

                # Drift Check alle 5 Sekunden
                if loop_count - last_drift_check > 50:
                    check_and_correct_drift(scf)
                    last_drift_check = loop_count

                # Sensorwerte
                d_front = current_ranges['front']
                d_back = current_ranges['back']
                d_left = current_ranges['left']
                d_right = current_ranges['right']

                vx = 0.0
                vy = 0.0
                yaw_rate = 0.0

                # Safety: Begrenzung auf 2m Radius
                dist_center = math.sqrt(current_pose['x']**2 + current_pose['y']**2)

                # Hysterese-Logik: Wenn wir ausweichen, erst bei mehr Platz wieder vorwärts
                obs_threshold = min_dist_mm + 200 if avoiding_obstacle else min_dist_mm

                if dist_center > 2.0:
                    avoiding_obstacle = False
                    angle_to_center = math.degrees(math.atan2(-current_pose['y'], -current_pose['x']))
                    curr_yaw = current_pose['yaw']
                    diff = (angle_to_center - curr_yaw + 180) % 360 - 180

                    vx = 0.1
                    yaw_rate = diff * 0.5

                elif d_front > obs_threshold:
                    avoiding_obstacle = False
                    # Vorwärts (Dynamische Geschwindigkeit)
                    # Skaliere Speed: Langsam bei 0.5m, Schnell bei >1.5m
                    # Formel: Linearer Anstieg zwischen min_dist und min_dist+1000mm
                    speed_factor = min(1.0, max(0.0, (d_front - min_dist_mm) / 1000.0))
                    vx = 0.15 + (fw_speed - 0.15) * speed_factor

                    # Wandverfolgung (Wall Following)
                    if d_left < 400:
                        yaw_rate = -15
                    elif d_right < 400:
                        yaw_rate = 15

                else:
                    avoiding_obstacle = True
                    # Hindernis - intelligente Drehung
                    vx = 0.0

                    # Entscheide Drehrichtung basierend auf ALLEN Sensoren
                    left_space = d_left + d_front * 0.5
                    right_space = d_right + d_front * 0.5

                    if left_space > right_space:
                        yaw_rate = rot_speed
                        vy = 0.15 # Seitlich ausweichen hilft beim Umfliegen
                    else:
                        yaw_rate = -rot_speed
                        vy = -0.15

                # Hindernisausweichung
                avoid_x, avoid_y = get_avoidance_vector()
                cmd_vx = vx + avoid_x
                cmd_vy = vy + avoid_y

                # Limiter
                max_vel = 0.8
                cmd_vx = np.clip(cmd_vx, -max_vel, max_vel)
                cmd_vy = np.clip(cmd_vy, -max_vel, max_vel)

                scf.cf.commander.send_hover_setpoint(cmd_vx, cmd_vy, yaw_rate, HEIGHT)

                if loop_count % 5 == 0:
                    update_plot()

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nAbbruch - Lande...")
            # Sanfte Landung
            h = HEIGHT
            while h > 0.05:
                scf.cf.commander.send_hover_setpoint(0, 0, 0, h)
                h -= 0.05
                time.sleep(0.1)

        finally:
            scf.cf.commander.send_stop_setpoint()
            ps.send_arming_request(False)
            is_flying = False

            process_final_map()
            logconf1.stop()
            logconf2.stop()
            logconf_bat.stop()
            try:
                logconf3.stop()
            except:
                pass


if __name__ == '__main__':
    run_rectangle_mission()
    plt.ioff()
    plt.show()