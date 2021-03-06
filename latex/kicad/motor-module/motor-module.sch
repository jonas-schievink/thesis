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
LIBS:cd4093
LIBS:motor-module-cache
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
L ICL7667 U2
U 1 1 5AD895AE
P 4250 3950
F 0 "U2" H 4190 4365 50  0000 R CNN
F 1 "TC426" H 4190 4290 50  0000 R CNN
F 2 "" H 4400 3100 50  0001 L CNN
F 3 "" H 4450 3650 50  0001 C CNN
	1    4250 3950
	0    -1   -1   0   
$EndComp
$Comp
L Conn_02x03_Top_Bottom J1
U 1 1 5AD89982
P 2150 2550
F 0 "J1" H 2200 2750 50  0000 C CNN
F 1 "Conn_02x03_Top_Bottom" H 2200 2350 50  0001 C CNN
F 2 "" H 2150 2550 50  0001 C CNN
F 3 "" H 2150 2550 50  0001 C CNN
	1    2150 2550
	0    -1   -1   0   
$EndComp
$Comp
L CD4093 U4
U 1 1 5AD8C238
P 4000 4700
F 0 "U4" H 4000 4900 50  0001 C CNN
F 1 "CD4093" H 4000 4500 50  0001 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 4000 4700 50  0001 C CNN
F 3 "" H 4000 4700 50  0001 C CNN
	1    4000 4700
	0    -1   -1   0   
$EndComp
$Comp
L CD4093 U5
U 1 1 5AD8EF1F
P 3500 5800
F 0 "U5" H 3500 6000 50  0000 C CNN
F 1 "CD4093" H 3500 5600 50  0001 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 3500 5800 50  0001 C CNN
F 3 "" H 3500 5800 50  0001 C CNN
	1    3500 5800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4000 4400 4150 4400
Wire Wire Line
	4150 4400 4150 4250
Wire Wire Line
	4350 4250 4350 4400
Wire Wire Line
	4350 4400 4500 4400
Wire Wire Line
	3400 5000 3400 5100
Wire Wire Line
	3400 5100 3900 5100
Wire Wire Line
	3600 5000 3600 5350
Wire Wire Line
	3900 5100 3900 5000
Connection ~ 3600 5100
Wire Wire Line
	4100 5000 4100 5100
Wire Wire Line
	4100 5100 4400 5100
Wire Wire Line
	4400 5100 4400 5000
Wire Wire Line
	4600 5000 4600 5100
Wire Wire Line
	4600 5100 5100 5100
Wire Wire Line
	4900 5000 4900 5250
Wire Wire Line
	5100 5100 5100 5000
Connection ~ 4900 5100
Wire Notes Line
	5300 4300 5300 5200
Wire Notes Line
	5300 5200 3200 5200
Wire Notes Line
	3200 5200 3200 4300
Wire Notes Line
	3200 4300 5300 4300
Wire Notes Line
	5300 5400 5300 6300
Wire Notes Line
	5300 6300 3200 6300
Wire Notes Line
	3200 6300 3200 5400
Wire Notes Line
	3200 5400 5300 5400
Wire Wire Line
	4750 5500 5000 5500
Wire Wire Line
	4750 6200 4750 5500
Wire Wire Line
	4400 6200 4750 6200
Wire Wire Line
	4600 6200 4600 6100
Wire Wire Line
	4400 6200 4400 6100
Connection ~ 4600 6200
Wire Wire Line
	3500 5500 3750 5500
Wire Wire Line
	3750 5500 3750 6200
Wire Wire Line
	3750 6200 4100 6200
Wire Wire Line
	3900 6200 3900 6100
Wire Wire Line
	4100 6200 4100 6100
Connection ~ 3900 6200
Wire Wire Line
	4500 5500 4500 5250
Wire Wire Line
	4500 5250 4900 5250
Wire Wire Line
	4000 5500 4000 5250
Wire Wire Line
	4000 5250 4250 5250
Wire Wire Line
	4250 5250 4250 5100
Connection ~ 4250 5100
Wire Wire Line
	4900 6100 4900 6200
Wire Wire Line
	4900 6200 5100 6200
Wire Wire Line
	5100 6200 5100 6100
Wire Wire Line
	3600 6200 3600 6100
Wire Wire Line
	3400 6200 3600 6200
Wire Wire Line
	3400 6200 3400 6100
Connection ~ 3500 6200
Connection ~ 5000 6200
$Comp
L +5V #PWR01
U 1 1 5AD91219
P 3750 3900
F 0 "#PWR01" H 3750 3750 50  0001 C CNN
F 1 "+5V" H 3750 4040 50  0000 C CNN
F 2 "" H 3750 3900 50  0001 C CNN
F 3 "" H 3750 3900 50  0001 C CNN
	1    3750 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 3950 3750 3950
Wire Wire Line
	3750 3950 3750 3900
$Comp
L GND #PWR02
U 1 1 5AD9132C
P 4750 4000
F 0 "#PWR02" H 4750 3750 50  0001 C CNN
F 1 "GND" H 4750 3850 50  0000 C CNN
F 2 "" H 4750 4000 50  0001 C CNN
F 3 "" H 4750 4000 50  0001 C CNN
	1    4750 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4000 4750 3950
Wire Wire Line
	4750 3950 4650 3950
Wire Wire Line
	3500 2500 3500 4400
$Comp
L Conn_02x03_Top_Bottom J2
U 1 1 5AD93D77
P 6400 2550
F 0 "J2" H 6450 2750 50  0000 C CNN
F 1 "Conn_02x03_Top_Bottom" H 6450 2350 50  0001 C CNN
F 2 "" H 6400 2550 50  0001 C CNN
F 3 "" H 6400 2550 50  0001 C CNN
	1    6400 2550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6300 2750 6600 2750
Connection ~ 6400 2750
Wire Wire Line
	6100 2250 6600 2250
Connection ~ 6400 2250
Wire Wire Line
	6600 2250 6600 2750
Connection ~ 6500 2750
Connection ~ 6500 2250
$Comp
L +BATT #PWR03
U 1 1 5AD94427
P 6100 2250
F 0 "#PWR03" H 6100 2100 50  0001 C CNN
F 1 "+BATT" H 6100 2390 50  0000 C CNN
F 2 "" H 6100 2250 50  0001 C CNN
F 3 "" H 6100 2250 50  0001 C CNN
	1    6100 2250
	1    0    0    -1  
$EndComp
$Comp
L Conn_02x03_Top_Bottom J3
U 1 1 5AD948E7
P 2150 6000
F 0 "J3" H 2200 6200 50  0000 C CNN
F 1 "Conn_02x03_Top_Bottom" H 2200 5800 50  0001 C CNN
F 2 "" H 2150 6000 50  0001 C CNN
F 3 "" H 2150 6000 50  0001 C CNN
	1    2150 6000
	0    -1   -1   0   
$EndComp
$Comp
L Conn_02x03_Top_Bottom J4
U 1 1 5AD95575
P 6400 6000
F 0 "J4" H 6450 6200 50  0000 C CNN
F 1 "Conn_02x03_Top_Bottom" H 6450 5800 50  0001 C CNN
F 2 "" H 6400 6000 50  0001 C CNN
F 3 "" H 6400 6000 50  0001 C CNN
	1    6400 6000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6300 5700 6600 5700
Connection ~ 6400 5700
Wire Wire Line
	6100 6200 6600 6200
Connection ~ 6400 6200
Wire Wire Line
	6600 6200 6600 5700
Connection ~ 6500 6200
Connection ~ 6500 5700
$Comp
L GND #PWR04
U 1 1 5AD960CC
P 6100 6200
F 0 "#PWR04" H 6100 5950 50  0001 C CNN
F 1 "GND" H 6100 6050 50  0000 C CNN
F 2 "" H 6100 6200 50  0001 C CNN
F 3 "" H 6100 6200 50  0001 C CNN
	1    6100 6200
	1    0    0    -1  
$EndComp
Connection ~ 6300 6200
Connection ~ 6300 2250
$Comp
L CD4093 U3
U 2 1 5AD9ACE1
P 4500 4700
F 0 "U3" H 4500 4900 50  0001 C CNN
F 1 "HEF4093B" H 4500 4500 50  0001 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 4500 4700 50  0001 C CNN
F 3 "" H 4500 4700 50  0001 C CNN
	2    4500 4700
	0    -1   -1   0   
$EndComp
$Comp
L CD4093 U4
U 3 1 5AD9B0C7
P 5000 4700
F 0 "U4" H 5000 4900 50  0001 C CNN
F 1 "CD4093" H 5000 4500 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 5000 4700 50  0001 C CNN
F 3 "" H 5000 4700 50  0001 C CNN
	3    5000 4700
	0    -1   -1   0   
$EndComp
$Comp
L CD4093 U3
U 4 1 5AD9B2DC
P 3500 4700
F 0 "U3" H 3500 4900 50  0000 C CNN
F 1 "HEF4093B" H 3500 4500 50  0001 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 3500 4700 50  0001 C CNN
F 3 "" H 3500 4700 50  0001 C CNN
	4    3500 4700
	0    -1   -1   0   
$EndComp
$Comp
L CD4093 U3
U 5 1 5AD9B663
P 5550 4700
F 0 "U3" H 5550 4900 50  0001 C CNN
F 1 "HEF4093B" H 5550 4500 50  0001 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 5550 4700 50  0001 C CNN
F 3 "" H 5550 4700 50  0001 C CNN
	5    5550 4700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR05
U 1 1 5AD9B87F
P 5550 4400
F 0 "#PWR05" H 5550 4250 50  0001 C CNN
F 1 "+5V" H 5550 4540 50  0000 C CNN
F 2 "" H 5550 4400 50  0001 C CNN
F 3 "" H 5550 4400 50  0001 C CNN
	1    5550 4400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 5AD9B8A1
P 5550 5000
F 0 "#PWR06" H 5550 4750 50  0001 C CNN
F 1 "GND" H 5550 4850 50  0000 C CNN
F 2 "" H 5550 5000 50  0001 C CNN
F 3 "" H 5550 5000 50  0001 C CNN
	1    5550 5000
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q3
U 1 1 5AD9E501
P 3850 3100
F 0 "Q3" H 4050 3150 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 4050 3050 50  0001 L CNN
F 2 "" H 4050 3200 50  0001 C CNN
F 3 "" H 3850 3100 50  0001 C CNN
	1    3850 3100
	1    0    0    1   
$EndComp
$Comp
L Q_NMOS_DGS Q1
U 1 1 5AD9E5CA
P 3850 2500
F 0 "Q1" H 4050 2550 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 4050 2450 50  0001 L CNN
F 2 "" H 4050 2600 50  0001 C CNN
F 3 "" H 3850 2500 50  0001 C CNN
	1    3850 2500
	1    0    0    1   
$EndComp
$Comp
L Q_NMOS_DGS Q4
U 1 1 5AD9E76D
P 4650 3100
F 0 "Q4" H 4850 3150 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 4850 3050 50  0001 L CNN
F 2 "" H 4850 3200 50  0001 C CNN
F 3 "" H 4650 3100 50  0001 C CNN
	1    4650 3100
	-1   0    0    1   
$EndComp
$Comp
L Q_NMOS_DGS Q2
U 1 1 5AD9E773
P 4650 2500
F 0 "Q2" H 4850 2550 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 4850 2450 50  0001 L CNN
F 2 "" H 4850 2600 50  0001 C CNN
F 3 "" H 4650 2500 50  0001 C CNN
	1    4650 2500
	-1   0    0    1   
$EndComp
Wire Wire Line
	4550 2700 4550 2900
Wire Wire Line
	3950 2700 3950 2900
Wire Wire Line
	3650 3100 3650 3550
Wire Wire Line
	3650 3550 4150 3550
Wire Wire Line
	4150 3550 4150 3650
Wire Wire Line
	4350 3650 4350 3550
Wire Wire Line
	4350 3550 4850 3550
Wire Wire Line
	4850 3550 4850 3100
Wire Wire Line
	3950 3300 4550 3300
Wire Wire Line
	4250 3300 4250 3350
Connection ~ 4250 3300
Wire Wire Line
	3950 2300 4550 2300
Wire Wire Line
	4250 2300 4250 2250
Connection ~ 4250 2300
$Comp
L +BATT #PWR07
U 1 1 5AD9EFD8
P 4250 2250
F 0 "#PWR07" H 4250 2100 50  0001 C CNN
F 1 "+BATT" H 4250 2390 50  0000 C CNN
F 2 "" H 4250 2250 50  0001 C CNN
F 3 "" H 4250 2250 50  0001 C CNN
	1    4250 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5AD9F21F
P 4250 3350
F 0 "#PWR08" H 4250 3100 50  0001 C CNN
F 1 "GND" H 4250 3200 50  0000 C CNN
F 2 "" H 4250 3350 50  0001 C CNN
F 3 "" H 4250 3350 50  0001 C CNN
	1    4250 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2800 4000 2800
Connection ~ 3950 2800
Wire Wire Line
	4550 2800 4500 2800
Connection ~ 4550 2800
$Comp
L Conn_01x01_Female A
U 1 1 5ADA0136
P 4200 2800
F 0 "A" H 4200 2900 50  0000 C CNN
F 1 "Conn_01x01_Female" H 4200 2700 50  0001 C CNN
F 2 "" H 4200 2800 50  0001 C CNN
F 3 "" H 4200 2800 50  0001 C CNN
	1    4200 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2500 3650 2500
$Comp
L Conn_01x01_Female B
U 1 1 5ADA0554
P 4300 2800
F 0 "B" H 4300 2900 50  0000 C CNN
F 1 "Conn_01x01_Female" H 4300 2700 50  0001 C CNN
F 2 "" H 4300 2800 50  0001 C CNN
F 3 "" H 4300 2800 50  0001 C CNN
	1    4300 2800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4850 2500 5000 2500
Wire Wire Line
	5000 2500 5000 4400
$Comp
L CD4093 U4
U 2 1 5ADA4015
P 5000 5800
F 0 "U4" H 5000 6000 50  0001 C CNN
F 1 "CD4093" H 5000 5600 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 5000 5800 50  0001 C CNN
F 3 "" H 5000 5800 50  0001 C CNN
	2    5000 5800
	0    -1   -1   0   
$EndComp
$Comp
L CD4093 U5
U 3 1 5ADA443D
P 4500 5800
F 0 "U5" H 4500 6000 50  0001 C CNN
F 1 "CD4093" H 4500 5600 50  0001 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 4500 5800 50  0001 C CNN
F 3 "" H 4500 5800 50  0001 C CNN
	3    4500 5800
	0    -1   -1   0   
$EndComp
$Comp
L CD4093 U6
U 4 1 5ADA48E8
P 4000 5800
F 0 "U6" H 4000 6000 50  0001 C CNN
F 1 "HEF4093B" H 4000 5600 50  0001 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 4000 5800 50  0001 C CNN
F 3 "" H 4000 5800 50  0001 C CNN
	4    4000 5800
	0    -1   -1   0   
$EndComp
$Comp
L CD4093 U6
U 5 1 5ADA4A26
P 5550 5800
F 0 "U6" H 5550 6000 50  0001 C CNN
F 1 "HEF4093B" H 5550 5600 50  0001 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 5550 5800 50  0001 C CNN
F 3 "" H 5550 5800 50  0001 C CNN
	5    5550 5800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5ADA4A75
P 5550 6100
F 0 "#PWR09" H 5550 5850 50  0001 C CNN
F 1 "GND" H 5550 5950 50  0000 C CNN
F 2 "" H 5550 6100 50  0001 C CNN
F 3 "" H 5550 6100 50  0001 C CNN
	1    5550 6100
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR010
U 1 1 5ADA4AAA
P 5550 5500
F 0 "#PWR010" H 5550 5350 50  0001 C CNN
F 1 "+5V" H 5550 5640 50  0000 C CNN
F 2 "" H 5550 5500 50  0001 C CNN
F 3 "" H 5550 5500 50  0001 C CNN
	1    5550 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 2750 2250 2850
Wire Wire Line
	2250 2850 3100 2850
Wire Wire Line
	3100 2850 3100 6350
Wire Wire Line
	3100 6350 3500 6350
Wire Wire Line
	3500 6350 3500 6200
Wire Wire Line
	2150 2750 2150 2950
Wire Wire Line
	2150 2950 3000 2950
Wire Wire Line
	3000 2950 3000 6450
Wire Wire Line
	3000 6450 5000 6450
Wire Wire Line
	5000 6450 5000 6200
Wire Wire Line
	4900 5500 4900 5350
Wire Wire Line
	4900 5350 3600 5350
Connection ~ 4900 5500
Text Label 2550 2950 0    60   ~ 0
DIR
Text Label 2550 2850 0    60   ~ 0
CTRL
$Comp
L L78L05_TO92 U1
U 1 1 5ADC721D
P 5900 3600
F 0 "U1" H 5750 3725 50  0000 C CNN
F 1 "78L05" H 5900 3725 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 5900 3825 50  0001 C CIN
F 3 "" H 5900 3550 50  0001 C CNN
	1    5900 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 5ADC7270
P 5900 3900
F 0 "#PWR011" H 5900 3650 50  0001 C CNN
F 1 "GND" H 5900 3750 50  0000 C CNN
F 2 "" H 5900 3900 50  0001 C CNN
F 3 "" H 5900 3900 50  0001 C CNN
	1    5900 3900
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR012
U 1 1 5ADC72A5
P 5400 3600
F 0 "#PWR012" H 5400 3450 50  0001 C CNN
F 1 "+BATT" H 5400 3740 50  0000 C CNN
F 2 "" H 5400 3600 50  0001 C CNN
F 3 "" H 5400 3600 50  0001 C CNN
	1    5400 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3600 5400 3600
$Comp
L +5V #PWR013
U 1 1 5ADC73A0
P 6400 3600
F 0 "#PWR013" H 6400 3450 50  0001 C CNN
F 1 "+5V" H 6400 3740 50  0000 C CNN
F 2 "" H 6400 3600 50  0001 C CNN
F 3 "" H 6400 3600 50  0001 C CNN
	1    6400 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 3600 6200 3600
Text Notes 5100 2550 0    60   ~ 0
High Side
Text Notes 5100 3150 0    60   ~ 0
Low Side
$EndSCHEMATC
