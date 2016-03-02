EESchema Schematic File Version 2
LIBS:power
LIBS:device
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
LIBS:sn754410
LIBS:lego_pf
LIBS:tsop1730
LIBS:baby_orangutan_b328
LIBS:BabyOrangutanAVR-cache
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
L SN754410 IC1
U 1 1 56D27A0F
P 4950 3250
F 0 "IC1" H 5000 4329 50  0000 C CNN
F 1 "SN754410" H 5000 4237 50  0000 C CNN
F 2 "HBridge-DIP16" H 4950 3400 50  0001 C CNN
F 3 "" H 4950 3250 60  0000 C CNN
	1    4950 3250
	1    0    0    -1  
$EndComp
$Comp
L Lego_PF P3
U 1 1 56D28305
P 2300 4400
F 0 "P3" V 2172 4628 50  0000 L CNN
F 1 "Lego_PF" V 2264 4628 50  0000 L CNN
F 2 "" H 2300 4400 50  0000 C CNN
F 3 "" H 2300 4400 50  0000 C CNN
	1    2300 4400
	0    1    1    0   
$EndComp
$Comp
L Lego_PF P5
U 1 1 56D2839F
P 3250 4400
F 0 "P5" V 3122 4628 50  0000 L CNN
F 1 "Lego_PF" V 3214 4628 50  0000 L CNN
F 2 "" H 3250 4400 50  0000 C CNN
F 3 "" H 3250 4400 50  0000 C CNN
	1    3250 4400
	0    1    1    0   
$EndComp
$Comp
L Lego_PF P6
U 1 1 56D2845C
P 4250 4350
F 0 "P6" V 4122 4578 50  0000 L CNN
F 1 "Lego_PF" V 4214 4578 50  0000 L CNN
F 2 "" H 4250 4350 50  0000 C CNN
F 3 "" H 4250 4350 50  0000 C CNN
	1    4250 4350
	0    1    1    0   
$EndComp
$Comp
L Lego_PF P7
U 1 1 56D284C2
P 5250 4350
F 0 "P7" V 5122 4578 50  0000 L CNN
F 1 "Lego_PF" V 5214 4578 50  0000 L CNN
F 2 "" H 5250 4350 50  0000 C CNN
F 3 "" H 5250 4350 50  0000 C CNN
	1    5250 4350
	0    1    1    0   
$EndComp
$Comp
L Led_Small D1
U 1 1 56D2851F
P 1700 3950
F 0 "D1" H 1700 3744 50  0000 C CNN
F 1 "Led_Small" H 1700 3836 50  0000 C CNN
F 2 "" V 1700 3950 50  0000 C CNN
F 3 "" V 1700 3950 50  0000 C CNN
	1    1700 3950
	-1   0    0    1   
$EndComp
$Comp
L Lego_PF P2
U 1 1 56D28734
P 1350 1700
F 0 "P2" H 1269 1318 50  0000 C CNN
F 1 "Lego_PF Battery In" H 1269 1410 50  0000 C CNN
F 2 "" H 1350 1700 50  0000 C CNN
F 3 "" H 1350 1700 50  0000 C CNN
	1    1350 1700
	-1   0    0    1   
$EndComp
$Comp
L TSOP1730 TSOP1
U 1 1 56D2884B
P 3550 1450
F 0 "TSOP1" V 3310 1878 50  0000 L CNN
F 1 "TSOP1730" V 3402 1878 50  0000 L CNN
F 2 "tsop-TSOP17XX" H 3550 1600 50  0001 C CNN
F 3 "" H 3550 1450 60  0000 C CNN
	1    3550 1450
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 56D28926
P 1450 3950
F 0 "R1" V 1242 3950 50  0000 C CNN
F 1 "R" V 1334 3950 50  0000 C CNN
F 2 "" V 1380 3950 50  0000 C CNN
F 3 "" H 1450 3950 50  0000 C CNN
	1    1450 3950
	0    1    1    0   
$EndComp
$Comp
L Baby_Orangutan_B328 P4
U 1 1 56D28AA2
P 2400 2850
F 0 "P4" H 2400 2161 50  0000 C CNN
F 1 "Baby_Orangutan_B328" H 2400 2069 50  0000 C CNN
F 2 "" H 2400 1650 50  0000 C CNN
F 3 "" H 2400 1650 50  0000 C CNN
	1    2400 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1550 3150 1550
Wire Wire Line
	3150 1550 3150 2300
Wire Wire Line
	1550 1850 3250 1850
Wire Wire Line
	3250 1850 3250 2400
Wire Wire Line
	3150 2400 3350 2400
Wire Wire Line
	1950 2100 3750 2100
Wire Wire Line
	3750 2100 3750 1750
Wire Wire Line
	3350 1750 3350 3750
Connection ~ 3250 2400
Wire Wire Line
	2450 3750 5400 3750
Connection ~ 3350 2400
Connection ~ 4750 3750
Connection ~ 4850 3750
Connection ~ 4950 3750
Wire Wire Line
	3150 2300 5650 2300
Wire Wire Line
	4950 2300 4950 2350
Wire Wire Line
	5050 2300 5050 2350
Connection ~ 4950 2300
Wire Wire Line
	4300 2300 4300 2850
Wire Wire Line
	4300 2850 4350 2850
Connection ~ 4300 2300
Wire Wire Line
	5650 2300 5650 2850
Connection ~ 5050 2300
Wire Wire Line
	5400 3750 5400 4150
Connection ~ 5050 3750
Wire Wire Line
	4400 4150 4400 3750
Connection ~ 4400 3750
Wire Wire Line
	3400 4200 3400 3750
Connection ~ 3400 3750
Wire Wire Line
	2450 4200 2450 3750
Connection ~ 3350 3750
Wire Wire Line
	2150 4200 2150 4050
Wire Wire Line
	2150 4050 5100 4050
Wire Wire Line
	3800 4050 3800 2300
Connection ~ 3800 2300
Wire Wire Line
	3100 4200 3100 4050
Connection ~ 3100 4050
Wire Wire Line
	4100 4050 4100 4150
Connection ~ 3800 4050
Wire Wire Line
	5100 4050 5100 4150
Connection ~ 4100 4050
Wire Wire Line
	5650 3350 5650 3900
Wire Wire Line
	5650 3900 5200 3900
Wire Wire Line
	5200 3900 5200 4150
Wire Wire Line
	5650 3250 5700 3250
Wire Wire Line
	5700 3250 5700 4000
Wire Wire Line
	5700 4000 5300 4000
Wire Wire Line
	5300 4000 5300 4150
Wire Wire Line
	4350 3350 4200 3350
Wire Wire Line
	4200 3350 4200 4150
Wire Wire Line
	4350 3250 4300 3250
Wire Wire Line
	4300 3250 4300 4150
Wire Wire Line
	1650 2400 1450 2400
Wire Wire Line
	1450 2400 1450 3750
Wire Wire Line
	1450 3750 2350 3750
Wire Wire Line
	2350 3750 2350 4200
Wire Wire Line
	1400 3800 1400 2300
Wire Wire Line
	1400 2300 1650 2300
Wire Wire Line
	2250 4200 2250 3800
Wire Wire Line
	2250 3800 1400 3800
Wire Wire Line
	3150 2500 3200 2500
Wire Wire Line
	3200 2500 3200 4200
Wire Wire Line
	3150 2600 3300 2600
Wire Wire Line
	3300 2600 3300 4200
Wire Wire Line
	1800 3950 2450 3950
Connection ~ 2450 3950
Wire Wire Line
	1300 3950 1300 3400
Wire Wire Line
	1300 3400 1650 3400
Wire Wire Line
	3150 3200 4150 3200
Wire Wire Line
	4150 3200 4150 2950
Wire Wire Line
	4150 2950 4350 2950
Wire Wire Line
	4350 3050 4100 3050
Wire Wire Line
	4100 3050 4100 3100
Wire Wire Line
	4100 3100 3150 3100
Wire Wire Line
	3150 3000 3950 3000
Wire Wire Line
	3950 3000 3950 2050
Wire Wire Line
	3950 2050 5900 2050
Wire Wire Line
	5900 2050 5900 2950
Wire Wire Line
	5900 2950 5650 2950
Wire Wire Line
	5650 3050 6050 3050
Wire Wire Line
	6050 3050 6050 2000
Wire Wire Line
	6050 2000 3900 2000
Wire Wire Line
	3900 2000 3900 2900
Wire Wire Line
	3900 2900 3150 2900
$Comp
L CONN_01X03 P1
U 1 1 56D66D85
P 950 2850
F 0 "P1" V 1027 2888 50  0000 L CNN
F 1 "UART" H 950 2600 50  0000 L CNN
F 2 "" H 950 2850 50  0000 C CNN
F 3 "" H 950 2850 50  0000 C CNN
	1    950  2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	1950 3950 1950 4100
Wire Wire Line
	1950 4100 1200 4100
Wire Wire Line
	1200 4100 1200 2950
Connection ~ 1950 3950
Wire Wire Line
	3550 1750 3550 1950
Wire Wire Line
	3550 1950 700  1950
Wire Wire Line
	700  1950 700  3350
Wire Wire Line
	700  3350 1550 3350
Wire Wire Line
	1550 3350 1550 3200
Wire Wire Line
	1550 3200 1650 3200
Wire Wire Line
	1650 3100 1350 3100
Wire Wire Line
	1350 3100 1350 2750
Wire Wire Line
	1350 2750 1150 2750
Wire Wire Line
	1650 3000 1300 3000
Wire Wire Line
	1300 3000 1300 2850
Wire Wire Line
	1300 2850 1150 2850
Wire Wire Line
	1200 2950 1150 2950
$Comp
L +9V #PWR?
U 1 1 56D67649
P 2000 1550
F 0 "#PWR?" H 2000 1400 50  0001 C CNN
F 1 "+9V" H 2018 1724 50  0000 C CNN
F 2 "" H 2000 1550 50  0000 C CNN
F 3 "" H 2000 1550 50  0000 C CNN
	1    2000 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56D67673
P 2050 1850
F 0 "#PWR?" H 2050 1600 50  0001 C CNN
F 1 "GND" H 2058 1676 50  0000 C CNN
F 2 "" H 2050 1850 50  0000 C CNN
F 3 "" H 2050 1850 50  0000 C CNN
	1    2050 1850
	1    0    0    -1  
$EndComp
$EndSCHEMATC
