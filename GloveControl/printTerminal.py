import sys

class PrintTerminal:
    def __init__(self):
        sys.stdout.write("\033[2J")

    def trigger_arm(self):
        """Hier kommt der tatsächliche Code rein, um die Drohne zu starten."""
        print("\n" + "!"*40)
        print(">>> ARMING SIGNAL AN DROHNE GESENDET <<<")
        print("!"*40 + "\n")
        
        # Hier würdest du z.B. mavlink.arm() aufrufen
        self.is_armed = True
        
    def calculate_distance(self, pos1, pos2):
            """Berechnet die euklidische Distanz zwischen zwei (x,y,z) Punkten."""
            # Formel: Wurzel((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
            dx = pos1[0] - pos2[0]
            dy = pos1[1] - pos2[1]
            dz = pos1[2] - pos2[2]
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            print(f"Abstand: {distance:.2f}")
            return math.sqrt(dx*dx + dy*dy + dz*dz)
    
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