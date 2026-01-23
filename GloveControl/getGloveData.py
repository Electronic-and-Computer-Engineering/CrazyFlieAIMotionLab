import socket
import struct
import time

class GloveData:
    def __init__(self, ip="0.0.0.0", port_map=None):
        # Standardkonfiguration, falls nichts übergeben wird
        if port_map is None:
            self.port_map = {40003: "GloveRight", 40004: "GloveLeft"}
        else:
            self.port_map = port_map
            
        self.udp_ip = ip
        self.buffer_size = 1024
        
        # Sockets initialisieren
        self.sockets = []
        self._init_sockets()
        
        # Speicher für Berechnungen (Zeit, X, Y, Z) für Speed
        self.object_states = {}
        
        # Speicher für die aktuellsten abrufbaren Daten
        self.current_data = {}
        
        # Initiale leere Daten setzen, damit man beim ersten Abruf keinen Fehler bekommt
        for name in self.port_map.values():
            self.current_data[name] = {
                "pos": (0, 0, 0),
                "speed": (0.0, 0.0, 0.0),
                "timestamp": 0
            }

    def _init_sockets(self):
        """Erstellt die Sockets und setzt sie auf non-blocking."""
        for port, name in self.port_map.items():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind((self.udp_ip, port))
                s.setblocking(0) # Wichtig: Non-blocking
                self.sockets.append((s, name))
                print(f"[Tracker] Port {port} ({name}) bereit.")
            except OSError as e:
                print(f"[Error] Konnte Port {port} nicht binden: {e}")

    def update(self):
        """
        Liest alle verfügbaren Pakete, leert den Puffer und aktualisiert den internen Status.
        Diese Funktion sollte regelmäßig (z.B. in einer Schleife) aufgerufen werden.
        """
        current_time = time.time()

        for sock, name in self.sockets:
            last_valid_data = None
            
            # Puffer leeren (Flush), nur das letzte Paket behalten
            try:
                while True:
                    data, _ = sock.recvfrom(self.buffer_size)
                    if len(data) == 14: # Validierung der Paketgröße
                        last_valid_data = data
            except BlockingIOError:
                pass # Puffer leer
            
            # Wenn neue Daten da waren, verarbeiten
            if last_valid_data:
                self._parse_and_store(last_valid_data, name, current_time)

    def _parse_and_store(self, data, name, current_time):
        try:
            values = struct.unpack('<7h', data)
            # Werte extrahieren (Index 0 ist ID, 1-3 sind Positionen)
            pos_x, pos_y, pos_z = values[1], values[2], values[3]
            
            speed_x, speed_y, speed_z = 0.0, 0.0, 0.0

            # Geschwindigkeitsberechnung
            if name in self.object_states:
                last_t, last_x, last_y, last_z = self.object_states[name]
                dt = current_time - last_t
                
                if dt > 0:
                    speed_x = (pos_x - last_x) / dt
                    speed_y = (pos_y - last_y) / dt
                    speed_z = (pos_z - last_z) / dt
            
            # State für nächsten Durchlauf speichern
            self.object_states[name] = (current_time, pos_x, pos_y, pos_z)
            
            # Public Data aktualisieren (das, was der User abruft)
            self.current_data[name] = {
                "pos": (pos_x, pos_y, pos_z),
                "speed": (speed_x, speed_y, speed_z),
                "timestamp": current_time
            }
            
        except struct.error:
            pass

    def get_data(self):
        """Gibt die aktuellsten Daten zurück. Ruft vorher update() auf."""
        self.update()
        return self.current_data

# Damit man es auch direkt testen kann (wie dein altes Skript)
if __name__ == "__main__":
    tracker = GloveData()
    print("Tracker läuft stand-alone... (Strg+C zum Beenden)")
    while True:
        data = tracker.get_data()
        print(f"Data: {data}")
        time.sleep(0.25)