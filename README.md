# ROS-Package `kurtberry_pi`

Enthält Teile von [`kurt_driver`](https://github.com/uos/kurt_driver).

## Ordnerstruktur

* `kernel`: Den Linux-Kernel betreffende Dateien (Device Tree Overlays, Treiber, etc.)
* `scripts`: Diverse Python-Scripts zum Testen

## Über das Setup

### LAN

Der LAN-Port des Pis kann benutzt werden, um ihn an ein größeres ROS-Setup
anzubinden. Ich habe ein sehr minimalistisches aber praktisches Netzwerksetup
eingestellt: Der Pi erstellt sich nur eine Link-Local IPv4-Adresse. So ist kein
manuelles Einstellen statischer IPs nötig, und einen DHCP-Server braucht man
auch nicht.

Natürlich wäre sehr unpraktisch, wenn man bei jedem Start die IP des Pis
herausfinden müsste. Dafür gibt es auch eine Lösung: [mDNS][]. Der Hostname des
Pis ist `kurtberry-pi`. Ist alles richtig eingestellt, sollte man den Pi als
`kurtberry-pi.local` ansprechen können.

Sollte das Setup nicht funktionieren, stelle sicher, dass Routing nach
`169.254.0.0/16` funktioniert (IPv4-Block für Link-Lokale Adressen).

Funktioniert das Anpingen der IP, aber nicht das Auflösen des Hostnamens, ist
vermutlich eine fehlerhafte mDNS-Konfiguration schuld. Das wird normalerweise
von Avahi oder systemd-networkd gemacht. Meines Wissens nach sollte das
Standard-Setup von Ubuntu et al. aber ausreichen (evtl. hilft es, testweise das
LAN-Interface im Network Manager auf "Link-Local only" zu stellen).

TODO: Zusätzlich statische IP einrichten, damit Integration einfacher (welche
IP, welches Netz?).

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

    ln -s ~/kurtberry-pi/ros ~/catkin_ws/src/kurtberry_pi

Ausführen:

    rosrun kurtberry_pi kurtberry_pi_node
