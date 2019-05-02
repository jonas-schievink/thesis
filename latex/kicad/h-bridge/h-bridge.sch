EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:h-bridge-cache
EELAYER 25 0
EELAYER END
$Descr User 4134 4000
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Q_NMOS_DGS Q1
U 1 1 5AD76678
P 1550 1100
F 0 "Q1" H 1750 1150 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 1750 1050 50  0001 L CNN
F 2 "" H 1750 1200 50  0001 C CNN
F 3 "" H 1550 1100 50  0001 C CNN
	1    1550 1100
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q3
U 1 1 5AD769C5
P 1550 1800
F 0 "Q3" H 1750 1850 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 1750 1750 50  0001 L CNN
F 2 "" H 1750 1900 50  0001 C CNN
F 3 "" H 1550 1800 50  0001 C CNN
	1    1550 1800
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q2
U 1 1 5AD76A37
P 2250 1100
F 0 "Q2" H 2450 1150 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 2450 1050 50  0001 L CNN
F 2 "" H 2450 1200 50  0001 C CNN
F 3 "" H 2250 1100 50  0001 C CNN
	1    2250 1100
	-1   0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q4
U 1 1 5AD76AC3
P 2250 1800
F 0 "Q4" H 2450 1850 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 2450 1750 50  0001 L CNN
F 2 "" H 2450 1900 50  0001 C CNN
F 3 "" H 2250 1800 50  0001 C CNN
	1    2250 1800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2150 1300 2150 1450
Wire Wire Line
	2150 1450 2150 1600
Connection ~ 2150 1450
Wire Wire Line
	1650 1300 1650 1450
Wire Wire Line
	1650 1450 1650 1600
Connection ~ 1650 1450
$Comp
L Conn_01x01_Female A
U 1 1 5AD7C886
P 1850 1450
F 0 "A" H 1850 1550 50  0000 C CNN
F 1 "Conn_01x01_Female" H 1850 1350 50  0001 C CNN
F 2 "" H 1850 1450 50  0001 C CNN
F 3 "" H 1850 1450 50  0001 C CNN
	1    1850 1450
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01_Female B
U 1 1 5AD7C99B
P 1950 1450
F 0 "B" H 1950 1550 50  0000 C CNN
F 1 "Conn_01x01_Female" H 1950 1350 50  0001 C CNN
F 2 "" H 1950 1450 50  0001 C CNN
F 3 "" H 1950 1450 50  0001 C CNN
	1    1950 1450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2150 900  1900 900 
Wire Wire Line
	1900 900  1650 900 
Wire Wire Line
	1900 900  1900 750 
Connection ~ 1900 900 
Wire Wire Line
	2150 2000 1900 2000
Wire Wire Line
	1900 2000 1650 2000
Wire Wire Line
	1900 2000 1900 2150
Connection ~ 1900 2000
$Comp
L +BATT #PWR?
U 1 1 5AD7D062
P 1900 750
F 0 "#PWR?" H 1900 600 50  0001 C CNN
F 1 "+BATT" H 1900 890 50  0000 C CNN
F 2 "" H 1900 750 50  0001 C CNN
F 3 "" H 1900 750 50  0001 C CNN
	1    1900 750 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5AD7D33A
P 1900 2150
F 0 "#PWR?" H 1900 1900 50  0001 C CNN
F 1 "GND" H 1900 2000 50  0000 C CNN
F 2 "" H 1900 2150 50  0001 C CNN
F 3 "" H 1900 2150 50  0001 C CNN
	1    1900 2150
	1    0    0    -1  
$EndComp
$EndSCHEMATC
