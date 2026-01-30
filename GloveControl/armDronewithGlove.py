import time
import math
from getGloveData import GloveData 
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.platformservice import PlatformService
from printTerminal import PrintTerminal

import sys
sys.path.append("../CrazyFlieAIMotionLab")
import cfMRB

URI = "radio://0/33/2M/E7E7E7E7AA"  # Brushless URI

ARMING_THRESHOLD = 200
RESET_THRESHOLD = 300

terminal = PrintTerminal()

class DroneManager:
    def __init__(self):
        self.is_armed = False
        self.ready_to_arm = True
        self.external_scf = None

    def trigger_arm(self):
        # Wenn wir eine externe Verbindung haben (vom Main-Loop), nutzen wir die!
        if self.external_scf is not None:
            self.send_arm_command(self.external_scf)
        else:
            # Sonst machen wir eine neue auf (wie in deinem alten Skript)
            cflib.crtp.init_drivers()
            with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
                self.send_arm_command(scf)


    def send_arm_command(self, scf_obj):
            ps = PlatformService(scf_obj.cf)
            
            if self.is_armed:
                ps.send_arming_request(False)
                terminal.addLine(">>> DISARMING SIGNAL AN DROHNE GESENDET <<<")
                self.is_armed = False

            else:
                ps.send_arming_request(True)
                terminal.addLine(">>> ARMING SIGNAL AN DROHNE GESENDET <<<")
                self.is_armed = True

        
    def calculate_distance(self, pos1, pos2):
            #Berechnet die euklidische Distanz zwischen zwei (x,y,z) Punkten.
            # Formel: Wurzel((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
            dx = pos1[0] - pos2[0]
            dy = pos1[1] - pos2[1]
            dz = pos1[2] - pos2[2]
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            terminal.addLine(f"Abstand: {distance:.2f}")
            return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def check_arming_condition(self, pos_left, pos_right):
        #Prüft Distanz und armed, falls Bedingungen erfüllt

        dist = self.calculate_distance(pos_left, pos_right)
        
        if dist < ARMING_THRESHOLD:
            if self.ready_to_arm:
                self.trigger_arm()
                self.ready_to_arm = False
                terminal.addLine(f"Abstand: {dist:.2f} (Warte auf > {RESET_THRESHOLD})")

        elif dist > RESET_THRESHOLD:
            if not self.ready_to_arm:
                terminal.addLine(f"Abstand {dist:.0f} > {RESET_THRESHOLD}. Bereit für neues Signal!")
                self.ready_to_arm = True
            
        return dist

def main():
    gloveTracker = GloveData() #tracker for Gloves
    drone = DroneManager()

    
    print("Hauptprogramm gestartet. Warte auf 'Clap' zum Armen...")

    try:
        while True:
            # 2. Daten abrufen
            data = gloveTracker.get_data()
            gloveRight = data["GloveRight"]
            rx, ry, rz = gloveRight["pos"]
            
            gloveLeft = data["GloveLeft"]
            lx, ly, lz = gloveLeft["pos"]

            # Wir brauchen BEIDE Handschuhe für die Distanzberechnung
            if not rx == 0 and not ry == 0 and not rz == 0 and not lx == 0 and not ly == 0 and not lz == 0:
                pos_r = gloveRight["pos"]
                pos_l = gloveLeft["pos"]
                
                # 3. Logik prüfen (via Funktionsaufruf)
                drone.check_arming_condition(pos_l, pos_r)
                
            else:
                 terminal.addLine("Warte auf Signal von beiden Handschuhen...")
            
            terminal.printAllLines()
            time.sleep(0.1) 

    except KeyboardInterrupt:
        print("Beendet.")

if __name__ == "__main__":
    main()