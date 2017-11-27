# Der KURTberry Pi

Das ist das Arbeits-Repository für meine Bachelorarbeit.

## Ordnerstruktur

* `kernel`: Den Linux-Kernel betreffende Dateien (Device Tree Overlays, Treiber, etc.)
* `node`: C++-Code, der später in einem ROS-Node landen soll - benutzt pigpio zur Ansteuerung der Motoren
* `scripts`: Diverse Python-Scripts zum Testen

## Über das Setup

### Ethernet

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
LAN-Interface im Network Manager auf "Link-Local only" zu stellen - dann tun
natürlich andere Anwendungen nicht mehr).

TODO: Zusätzlich statische IP einrichten, damit Integration einfacher (welche
IP, welches Netz?).

[mDNS]: https://en.wikipedia.org/wiki/Multicast_DNS

### Encodertreiber (Device Tree)

Der Plan ist, die Encoder über Linux' eingebauten `rotary-encoder` Treiber
laufen zu lassen. Dazu muss ein Device Tree Overlay geschrieben werden.

Leider ist der Kernel zu alt (4.4) bzw. das `dtc`-Tool inkompatibel mit dem vom
Kernel benötigten. Man müsste also `dtc` selbst kompilieren.
