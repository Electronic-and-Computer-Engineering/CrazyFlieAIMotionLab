import time
import math
from getGloveData import GloveData 

# --- Kofiguration ---
# Hier musst du testen, welcher Wert passt. 
# Wenn die Einheiten Millimeter sind, wären 200 = 20cm.
ARMING_THRESHOLD = 50 

class DroneManager:
    def __init__(self):
        self.is_armed = False

    def calculate_distance(self, pos1, pos2):
        """Berechnet die euklidische Distanz zwischen zwei (x,y,z) Punkten."""
        # Formel: Wurzel((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        dz = pos1[2] - pos2[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        print(f"Abstand: {distance:.2f}")
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def trigger_arm(self):
        """Hier kommt der tatsächliche Code rein, um die Drohne zu starten."""
        print("\n" + "!"*40)
        print(">>> ARMING SIGNAL AN DROHNE GESENDET <<<")
        print("!"*40 + "\n")
        
        # Hier würdest du z.B. mavlink.arm() aufrufen
        self.is_armed = True

    def check_arming_condition(self, pos_left, pos_right):
        """Prüft Distanz und armed, falls Bedingungen erfüllt."""
        dist = self.calculate_distance(pos_left, pos_right)
        
        # Nur armen, wenn noch NICHT gearmed ist UND Distanz klein genug
        if not self.is_armed:
            print(f"Abstand: {dist:.2f} (Warte auf < {ARMING_THRESHOLD})")
            if dist < ARMING_THRESHOLD:
                self.trigger_arm()
        else:
            # Drohne ist schon an - hier könnte man Disarm-Logik einbauen
            pass
            
        return dist

def main():
    # 1. Tracker und Manager initialisieren
    tracker = GloveData()
    drone = DroneManager()
    
    print("Hauptprogramm gestartet. Warte auf 'Clap' zum Armen...")

    try:
        while True:
            # 2. Daten abrufen
            data = tracker.get_data()

            drone.calculate_distance()

            # Wir brauchen BEIDE Handschuhe für die Distanzberechnung
            if "GloveRight" in data and "GloveLeft" in data:
                pos_r = data["GloveRight"]["pos"]
                pos_l = data["GloveLeft"]["pos"]
                
                # 3. Logik prüfen (via Funktionsaufruf)
                drone.check_arming_condition(pos_l, pos_r)
                
            else:
                print("Warte auf Signal von beiden Handschuhen...")

            time.sleep(0.1) 

    except KeyboardInterrupt:
        print("Beendet.")

if __name__ == "__main__":
    main()