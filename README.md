# ROS-Package `kurtberry_pi`

## Ordnerstruktur

* `cmake`: `FindX.cmake`-Skripte zum Auffinden benötigter Bibliotheken.
* `kernel`: Den Linux-Kernel betreffende Dateien (Device Tree Overlays, Treiber, etc.).
* `launch`: Enthält Launch-Files zum Starten und Konfigurieren des gesamten Projekts.
* `meshes`: Enthält 3D-Meshes zur Visualisierung mit [rviz](http://wiki.ros.org/rviz) oder [Gazebo](http://gazebosim.org/) (kopiert von [`kurt_driver`](https://github.com/uos/kurt_driver)).
* `scripts`: Diverse Python-Scripts zum Testen der Hardware und Plotten des Systemverhaltens.
* `src`: Eigentlicher Code des ROS-Nodes, der auf dem Raspberry Pi läuft.
* `urdf`: Roboterbeschreibungsdateien (größtenteils kopiert von [`kurt_driver`](https://github.com/uos/kurt_driver)).

## Über das Setup

### LAN

Der LAN-Port des Pis kann benutzt werden, um ihn an ein größeres ROS-Setup
anzubinden. Der Pi ist `192.168.100.1/24` und hat zusätzlich [mDNS][] um als
`kurtberry-pi.local` erreichbar zu sein.

SSH-Zugang: Nutzer `pi`, Passwort `kurtberryp1` (oder `kurtberry-p1`, siehe
Zettel im Roboter).

[mDNS]: https://en.wikipedia.org/wiki/Multicast_DNS

### Encodertreiber (Device Tree)

Der Plan ist, die Encoder über Linux' eingebauten `rotary-encoder` Treiber
laufen zu lassen. Dazu muss ein Device Tree Overlay geschrieben werden.

Leider ist das `dtc`-Tool aus den Repos inkompatibel mit dem vom Kernel
benötigten. Deswegen wurde es aus den Raspberry Pi [Kernelquellen] (Commit
[`e80a8a55ba8512b531c9447d1307378bccc98a40`][kernel-commit], Branch `rpi-4.9.y`)
kompiliert und nach `/usr/local/bin` installiert. Bei Bedarf kann das mit
`make ARCH=arm dtbs` reproduziert werden. Dieser Schritt wird hoffentlich nicht
mehr nötig sein, wenn das `device-tree-compiler`-Paket aktualisiert wurde.

[Kernelquellen]: https://github.com/raspberrypi/linux
[kernel-commit]: https://github.com/raspberrypi/linux/commit/e80a8a55ba8512b531c9447d1307378bccc98a40

## Workflow

Nachdem man über LAN mit dem Pi verbunden ist, kann man folgenden Befehl
ausführen, um das `kurtberry-pi` Repo auf dem Pi lokal einzubinden:

    sshfs kurtberry-pi pi@kurtberry-pi.local:kurtberry-pi

Jetzt kann man normal im `kurtberry-pi` Ordner editieren und git nutzen. Zum
Kompilieren kann man eine SSH-Sitzung öffnen.

### ROS

Das ROS-Package wurde in den Catkin-Workspace `~/catkin_ws` gelinkt:

    ln -s ~/kurtberry-pi ~/catkin_ws/src/kurtberry_pi

Zum Bauen:

    cd ~/catkin_ws
    catkin b kurtberry_pi

Eventuell muss die Job-Anzahl reduziert werden damit der RAM ausreicht:

    catkin b kurtberry_pi -j 1

Zum Ausführen (aufgrund des GPIO-Zugriffs werden Root-Rechte benötigt, daher
öffne ich hierfür meistens ein zusätzliches Root-Terminal):

    sudo -s
    roslaunch kurtberry_pi kurt.launch

Alternativ ermöglicht das Launchfile `kurt-ps3.launch` die Fernsteuerung des
Roboters mit einem per Bluetooth verbundenen PS3-Controller.

## Copyright

Enthält Teile von [`kurt_driver`](https://github.com/uos/kurt_driver).

`kurt_driver` © Jochen Sprickerhof
