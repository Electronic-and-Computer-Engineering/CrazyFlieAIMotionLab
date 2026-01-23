import socket
import struct
import time

UDP_IP = "0.0.0.0"
OBJECT_MAP = { 40003: "GloveRight", 40004: "GloveLeft"}
BUFFER_SIZE = 1024 

# Zustandsspeicher
object_states = {}

def main():
    sockets = []
    print("Starte 0.25s Interval Listener (Buffer Flushing)...")
    
    for port, name in OBJECT_MAP.items():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((UDP_IP, port))
        
        # WICHTIG: Socket auf "Nicht-Blockierend" stellen.
        # Das erlaubt uns, den Puffer leer zu räumen, ohne dass das Programm stehen bleibt.
        s.setblocking(0) 
        
        sockets.append((s, name)) # Wir speichern Tupel (Socket, Name)
        print(f" -> Port {port} ({name}) bereit.")

    print("-" * 60)

    try:
        while True:
            # 1. Das gewünschte Intervall warten
            time.sleep(0.25)

            # 2. Alle Sockets abfragen
            for sock, name in sockets:
                last_valid_data = None
                
                # --- PUFFER LEEREN (FLUSH) ---
                # Wir lesen so lange Pakete aus dem Speicher, bis keiner mehr da ist.
                # Wir behalten nur das allerletzte Paket (das aktuellste).
                try:
                    while True:
                        data, _ = sock.recvfrom(BUFFER_SIZE)
                        if len(data) == 14:
                            last_valid_data = data
                except BlockingIOError:
                    # Das passiert, wenn der Puffer leer ist -> gut so, weitermachen!
                    pass

                # 3. Wenn ein Paket da war, berechnen wir JETZT
                if last_valid_data:
                    parse_packet(last_valid_data, name)

    except KeyboardInterrupt:
        print("\nBeendet.")

def parse_packet(data, obj_name):
    current_time = time.time() # Exakte Zeit nehmen

    try:
        values = struct.unpack('<7h', data)
        # Werte extrahieren
        raw_id = values[0] 
        pos_x = values[1] 
        pos_y = values[2] 
        pos_z = values[3] 
        
        # Speed Init
        speed_x, speed_y, speed_z = 0.0, 0.0, 0.0

        if obj_name in object_states:
            last_t, last_x, last_y, last_z = object_states[obj_name]
            
            dt = current_time - last_t
            
            # Berechnung nur wenn Zeit vergangen ist
            if dt > 0:
                speed_x = (pos_x - last_x) / dt 
                speed_y = (pos_y - last_y) / dt 
                speed_z = (pos_z - last_z) / dt 

        # Speichern
        object_states[obj_name] = (current_time, pos_x, pos_y, pos_z)

        # Ausgabe
        print(f"[{obj_name}] Pos: {pos_x}, {pos_y}, {pos_z} | Speed: {speed_x:.2f}, {speed_y:.2f}, {speed_z:.2f}")

    except struct.error:
        pass

if __name__ == "__main__":
    main()