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
LIBS:traco
LIBS:electronics-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
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
L 74HC245 U1
U 1 1 5B254A63
P 6400 4050
F 0 "U1" H 6500 4625 50  0000 L BNN
F 1 "74HC245" H 6450 3475 50  0000 L TNN
F 2 "Housings_DIP:DIP-20_W7.62mm" H 6400 4050 50  0001 C CNN
F 3 "" H 6400 4050 50  0001 C CNN
	1    6400 4050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR10
U 1 1 5B254AFC
P 5550 4650
F 0 "#PWR10" H 5550 4400 50  0001 C CNN
F 1 "GND" H 5550 4500 50  0000 C CNN
F 2 "" H 5550 4650 50  0001 C CNN
F 3 "" H 5550 4650 50  0001 C CNN
	1    5550 4650
	1    0    0    -1  
$EndComp
NoConn ~ 7100 4250
NoConn ~ 7100 4150
NoConn ~ 7100 4050
NoConn ~ 7100 3950
NoConn ~ 5700 4250
NoConn ~ 5700 4150
NoConn ~ 5700 4050
NoConn ~ 5700 3950
$Comp
L R R2
U 1 1 5B255220
P 5400 4100
F 0 "R2" V 5480 4100 50  0000 C CNN
F 1 "100K" V 5400 4100 50  0000 C CNN
F 2 "" V 5330 4100 50  0001 C CNN
F 3 "" H 5400 4100 50  0001 C CNN
	1    5400 4100
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5B2552AF
P 5200 4100
F 0 "R1" V 5280 4100 50  0000 C CNN
F 1 "100K" V 5200 4100 50  0000 C CNN
F 2 "" V 5130 4100 50  0001 C CNN
F 3 "" H 5200 4100 50  0001 C CNN
	1    5200 4100
	1    0    0    -1  
$EndComp
$Comp
L Screw_Terminal_01x02 J1
U 1 1 5B255975
P 3600 2450
F 0 "J1" H 3600 2550 50  0000 C CNN
F 1 "Battery Connector" H 3600 2250 50  0001 C CNN
F 2 "" H 3600 2450 50  0001 C CNN
F 3 "" H 3600 2450 50  0001 C CNN
	1    3600 2450
	-1   0    0    -1  
$EndComp
$Comp
L TEN10-2411 PS1
U 1 1 5B259214
P 5400 2500
F 0 "PS1" H 5400 2850 60  0000 C CNN
F 1 "TEN10-2411" H 5400 2150 60  0000 C CNN
F 2 "" H 5400 2875 60  0001 C CNN
F 3 "" H 5400 2875 60  0001 C CNN
	1    5400 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR1
U 1 1 5B25948A
P 4150 2350
F 0 "#PWR1" H 4150 2100 50  0001 C CNN
F 1 "GND" H 4150 2200 50  0000 C CNN
F 2 "" H 4150 2350 50  0001 C CNN
F 3 "" H 4150 2350 50  0001 C CNN
	1    4150 2350
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR3
U 1 1 5B2595B5
P 3900 2650
F 0 "#PWR3" H 3900 2500 50  0001 C CNN
F 1 "+BATT" H 3900 2790 50  0000 C CNN
F 2 "" H 3900 2650 50  0001 C CNN
F 3 "" H 3900 2650 50  0001 C CNN
	1    3900 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 5B259911
P 6550 2350
F 0 "#PWR2" H 6550 2100 50  0001 C CNN
F 1 "GND" H 6550 2200 50  0000 C CNN
F 2 "" H 6550 2350 50  0001 C CNN
F 3 "" H 6550 2350 50  0001 C CNN
	1    6550 2350
	1    0    0    -1  
$EndComp
Text GLabel 4350 4200 3    60   Input ~ 0
GPIO5
Text GLabel 4500 4200 3    60   Input ~ 0
GPIO6
Text GLabel 4650 4200 3    60   Input ~ 0
GPIO22
Text GLabel 4800 4200 3    60   Input ~ 0
GPIO23
Text GLabel 7500 2600 1    60   Input ~ 0
R_CTRL
Text GLabel 7600 2600 1    60   Input ~ 0
R_DIR
Text GLabel 7150 2600 1    60   Input ~ 0
L_CTRL
Text GLabel 7250 2600 1    60   Input ~ 0
L_DIR
$Comp
L CP C2
U 1 1 5B25A464
P 6150 2500
F 0 "C2" H 6175 2600 50  0000 L CNN
F 1 "1000µF" H 6175 2400 50  0000 L CNN
F 2 "" H 6188 2350 50  0001 C CNN
F 3 "" H 6150 2500 50  0001 C CNN
	1    6150 2500
	1    0    0    1   
$EndComp
$Comp
L CP C1
U 1 1 5B25A6CA
P 4350 2500
F 0 "C1" H 4375 2600 50  0000 L CNN
F 1 "300µF" H 4375 2400 50  0000 L CNN
F 2 "" H 4388 2350 50  0001 C CNN
F 3 "" H 4350 2500 50  0001 C CNN
	1    4350 2500
	1    0    0    1   
$EndComp
$Comp
L GND #PWR8
U 1 1 5B25BDAA
P 7350 4400
F 0 "#PWR8" H 7350 4150 50  0001 C CNN
F 1 "GND" H 7350 4250 50  0000 C CNN
F 2 "" H 7350 4400 50  0001 C CNN
F 3 "" H 7350 4400 50  0001 C CNN
	1    7350 4400
	1    0    0    -1  
$EndComp
$Comp
L C_Small C3
U 1 1 5B25BDC5
P 7350 4300
F 0 "C3" H 7360 4370 50  0000 L CNN
F 1 "100nF" H 7360 4220 50  0000 L CNN
F 2 "" H 7350 4300 50  0001 C CNN
F 3 "" H 7350 4300 50  0001 C CNN
	1    7350 4300
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG2
U 1 1 5B25C422
P 9050 5300
F 0 "#FLG2" H 9050 5375 50  0001 C CNN
F 1 "PWR_FLAG" H 9050 5450 50  0000 C CNN
F 2 "" H 9050 5300 50  0001 C CNN
F 3 "" H 9050 5300 50  0001 C CNN
	1    9050 5300
	-1   0    0    1   
$EndComp
$Comp
L +BATT #PWR12
U 1 1 5B25C43F
P 9050 5300
F 0 "#PWR12" H 9050 5150 50  0001 C CNN
F 1 "+BATT" H 9050 5440 50  0000 C CNN
F 2 "" H 9050 5300 50  0001 C CNN
F 3 "" H 9050 5300 50  0001 C CNN
	1    9050 5300
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR7
U 1 1 5B25CFCD
P 7350 4200
F 0 "#PWR7" H 7350 4050 50  0001 C CNN
F 1 "+5V" H 7350 4340 50  0000 C CNN
F 2 "" H 7350 4200 50  0001 C CNN
F 3 "" H 7350 4200 50  0001 C CNN
	1    7350 4200
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR9
U 1 1 5B25D25E
P 5550 4450
F 0 "#PWR9" H 5550 4300 50  0001 C CNN
F 1 "+5V" H 5550 4590 50  0000 C CNN
F 2 "" H 5550 4450 50  0001 C CNN
F 3 "" H 5550 4450 50  0001 C CNN
	1    5550 4450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR4
U 1 1 5B25D281
P 6750 2650
F 0 "#PWR4" H 6750 2500 50  0001 C CNN
F 1 "+5V" H 6750 2790 50  0000 C CNN
F 2 "" H 6750 2650 50  0001 C CNN
F 3 "" H 6750 2650 50  0001 C CNN
	1    6750 2650
	1    0    0    -1  
$EndComp
Text Notes 7000 2200 0    60   ~ 0
Zu Motortreibern
Text Notes 3700 4750 0    60   ~ 0
Zu Raspberry Pi
$Comp
L +5V #PWR5
U 1 1 5B25F61E
P 3450 4050
F 0 "#PWR5" H 3450 3900 50  0001 C CNN
F 1 "+5V" H 3450 4190 50  0000 C CNN
F 2 "" H 3450 4050 50  0001 C CNN
F 3 "" H 3450 4050 50  0001 C CNN
	1    3450 4050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR6
U 1 1 5B25F641
P 3600 4150
F 0 "#PWR6" H 3600 3900 50  0001 C CNN
F 1 "GND" H 3600 4000 50  0000 C CNN
F 2 "" H 3600 4150 50  0001 C CNN
F 3 "" H 3600 4150 50  0001 C CNN
	1    3600 4150
	-1   0    0    1   
$EndComp
Text GLabel 3450 4200 3    60   Input ~ 0
5V
Text GLabel 3600 4200 3    60   Input ~ 0
GND
$Comp
L Conn_01x05 J2
U 1 1 5B25FA89
P 3350 3150
F 0 "J2" H 3350 3450 50  0000 C CNN
F 1 "Encoder (L)" V 3450 3150 50  0000 C CNN
F 2 "" H 3350 3150 50  0001 C CNN
F 3 "" H 3350 3150 50  0001 C CNN
	1    3350 3150
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x05 J3
U 1 1 5B25FAF3
P 4600 3150
F 0 "J3" H 4600 3450 50  0000 C CNN
F 1 "Encoder (R)" V 4700 3150 50  0000 C CNN
F 2 "" H 4600 3150 50  0001 C CNN
F 3 "" H 4600 3150 50  0001 C CNN
	1    4600 3150
	1    0    0    -1  
$EndComp
Text GLabel 3750 4200 3    60   Input ~ 0
GPIO11
Text GLabel 3900 4200 3    60   Input ~ 0
GPIO10
Text GLabel 4050 4200 3    60   Input ~ 0
GPIO21
Text GLabel 4200 4200 3    60   Input ~ 0
GPIO20
Text GLabel 3300 4200 3    60   Input ~ 0
3,3V
Text Label 3750 3850 1    60   ~ 0
LEFT_A
Text Label 3900 3850 1    60   ~ 0
LEFT_B
Text Label 4050 3850 1    60   ~ 0
RIGHT_A
Text Label 4200 3850 1    60   ~ 0
RIGHT_B
Wire Wire Line
	5200 4550 5700 4550
Wire Wire Line
	5550 4450 5700 4450
Wire Wire Line
	4350 3550 5700 3550
Wire Wire Line
	5400 3950 5400 3550
Connection ~ 5400 3550
Wire Wire Line
	4650 3750 5700 3750
Wire Wire Line
	5200 3950 5200 3750
Connection ~ 5200 3750
Wire Wire Line
	5200 4250 5200 4550
Wire Wire Line
	5400 4250 5400 4550
Connection ~ 5550 4550
Connection ~ 5400 4550
Wire Wire Line
	7100 3550 7500 3550
Wire Wire Line
	7500 3550 7500 2600
Wire Wire Line
	7100 3650 7600 3650
Wire Wire Line
	7600 3650 7600 2600
Wire Wire Line
	7100 3750 7150 3750
Wire Wire Line
	7150 3750 7150 2600
Wire Wire Line
	7100 3850 7250 3850
Wire Wire Line
	7250 3850 7250 2600
Wire Wire Line
	6100 2350 6550 2350
Wire Wire Line
	6100 2650 6750 2650
Wire Wire Line
	3800 2550 3800 2650
Wire Wire Line
	3800 2650 4700 2650
Wire Wire Line
	4700 2650 4700 2600
Wire Wire Line
	3800 2450 3800 2350
Wire Wire Line
	3800 2350 4700 2350
Wire Wire Line
	4700 2350 4700 2400
Connection ~ 3900 2650
Connection ~ 4350 2650
Connection ~ 4350 2350
Connection ~ 4150 2350
Connection ~ 6150 2350
Connection ~ 6150 2650
Wire Wire Line
	5700 3850 4800 3850
Wire Wire Line
	4800 3850 4800 4200
Wire Wire Line
	4650 3750 4650 4200
Wire Wire Line
	5700 3650 4500 3650
Wire Wire Line
	4500 3650 4500 4200
Wire Wire Line
	4350 3550 4350 4200
Wire Wire Line
	5550 4650 5550 4550
Wire Wire Line
	3600 4200 3600 4150
Wire Wire Line
	3450 4200 3450 4050
Wire Wire Line
	3750 4200 3750 3150
Wire Wire Line
	3900 4200 3900 3350
Wire Wire Line
	4050 4200 4050 3150
Wire Wire Line
	4200 4200 4200 3350
$Comp
L GND #PWR?
U 1 1 5B266263
P 3000 2750
F 0 "#PWR?" H 3000 2500 50  0001 C CNN
F 1 "GND" H 3000 2600 50  0000 C CNN
F 2 "" H 3000 2750 50  0001 C CNN
F 3 "" H 3000 2750 50  0001 C CNN
	1    3000 2750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5B266838
P 9500 5300
F 0 "#PWR?" H 9500 5150 50  0001 C CNN
F 1 "+3.3V" H 9500 5440 50  0000 C CNN
F 2 "" H 9500 5300 50  0001 C CNN
F 3 "" H 9500 5300 50  0001 C CNN
	1    9500 5300
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG?
U 1 1 5B266868
P 9500 5300
F 0 "#FLG?" H 9500 5375 50  0001 C CNN
F 1 "PWR_FLAG" H 9500 5450 50  0000 C CNN
F 2 "" H 9500 5300 50  0001 C CNN
F 3 "" H 9500 5300 50  0001 C CNN
	1    9500 5300
	-1   0    0    1   
$EndComp
Wire Wire Line
	3550 2950 4400 2950
Wire Wire Line
	3000 2750 3650 2750
Wire Wire Line
	3650 2750 3650 2950
Connection ~ 3650 2950
NoConn ~ 4400 3050
NoConn ~ 3550 3050
Wire Wire Line
	4050 3150 4400 3150
Wire Wire Line
	3750 3150 3550 3150
Wire Wire Line
	3900 3350 3550 3350
Wire Wire Line
	4200 3350 4400 3350
Wire Wire Line
	4400 3250 3550 3250
$Comp
L +3.3V #PWR?
U 1 1 5B2954E4
P 3900 3250
F 0 "#PWR?" H 3900 3100 50  0001 C CNN
F 1 "+3.3V" H 3900 3390 50  0000 C CNN
F 2 "" H 3900 3250 50  0001 C CNN
F 3 "" H 3900 3250 50  0001 C CNN
	1    3900 3250
	1    0    0    -1  
$EndComp
Connection ~ 3900 3250
$Comp
L +3.3V #PWR?
U 1 1 5B2955FC
P 3300 3800
F 0 "#PWR?" H 3300 3650 50  0001 C CNN
F 1 "+3.3V" H 3300 3940 50  0000 C CNN
F 2 "" H 3300 3800 50  0001 C CNN
F 3 "" H 3300 3800 50  0001 C CNN
	1    3300 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 3800 3300 4200
$EndSCHEMATC
