# Testskripte und Tools

Inhalt:

* `motor.py`: Steuerung eines KURT-Motors (H-Brücke) über die GPIOs (Geschwindigkeit mit Tastatur einstellbar)
* `encoder.py`: Auslesen eines Encoders via GPIO (nicht evdev)
* `plotticks.py`: Erhöht die Motorgeschwindigkeit und gibt dabei Geschwindigkeit und Encoderticks/Sekunde als CSV aus
* `plotodom.py`: Sendet Geschwindigkeiten an `/cmd_vel` und zeichnet die gemeldete Geschwindigkeit aus `/odom` auf. Der Kurt-Node muss dabei laufen. Die Motoren müssen angeschlossen sein, dürfen aber nicht mit den Rädern verbunden sein. Die aufgezeichneten Daten werden in CSV-Dateien gespeichert.
* `draw.py`: Rendert einen Graphen aus den mit `plotodom.py` gespeicherten CSV-Dateien.
