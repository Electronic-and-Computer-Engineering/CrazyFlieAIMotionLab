import time
# Importieren der Klasse aus unserem neuen File
from getGloveData import GloveData 

def main():
    # 1. Tracker initialisieren
    tracker = GloveData()
    
    print("Hauptprogramm gestartet. Lese Daten...")

    try:
        while True:
            # 2. Daten abrufen (das macht intern den Puffer-Flush und die Berechnung)
            data = tracker.get_data()

            # 3. Daten verwenden
            gloveRight = data["GloveRight"]
            print(f"Data: {gloveRight}")

            # Beispielzugriff:
            #rx, ry, rz = gloveRight["pos"]
            #rspeed = gloveRight["speed"]

            time.sleep(0.01) 

    except KeyboardInterrupt:
        print("Beendet.")

if __name__ == "__main__":
    main()