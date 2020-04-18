EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Control box for Teensylev V2"
Date "2019-09-20"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L teensy:Teensy3.2 U1
U 1 1 5D90D2EC
P 8850 3650
F 0 "U1" H 8850 5300 60  0000 C CNN
F 1 "Teensy3.2" H 8850 5181 60  0000 C CNN
F 2 "" H 8850 2900 60  0000 C CNN
F 3 "" H 8850 2900 60  0000 C CNN
	1    8850 3650
	1    0    0    -1  
$EndComp
$Comp
L TeensyLev:Pololu_D24V10F5 U5
U 1 1 5D919B3E
P 3000 1250
F 0 "U5" H 3000 850 50  0000 C CNN
F 1 "Pololu D24V10F5" H 3025 976 50  0000 C CNN
F 2 "" H 3050 850 50  0001 C CNN
F 3 "" H 3050 850 50  0001 C CNN
	1    3000 1250
	-1   0    0    1   
$EndComp
$Comp
L TeensyLev:Pololu_TB6612FNG U2
U 1 1 5D916462
P 5150 2850
F 0 "U2" H 5100 2250 50  0000 C CNN
F 1 "Pololu TB6612FNG" H 5125 2376 50  0000 C CNN
F 2 "" H 5000 2900 50  0001 C CNN
F 3 "" H 5000 2900 50  0001 C CNN
	1    5150 2850
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5D91DED7
P 5650 2300
F 0 "#PWR?" H 5650 2150 50  0001 C CNN
F 1 "+3.3V" H 5665 2473 50  0000 C CNN
F 2 "" H 5650 2300 50  0001 C CNN
F 3 "" H 5650 2300 50  0001 C CNN
	1    5650 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 2300 5650 2500
Wire Wire Line
	5650 2500 5600 2500
Wire Wire Line
	5600 2600 5650 2600
Wire Wire Line
	5650 2600 5650 2500
Connection ~ 5650 2500
Wire Wire Line
	5600 2700 5650 2700
Wire Wire Line
	5650 2700 5650 2600
Connection ~ 5650 2600
Wire Wire Line
	5600 2800 5650 2800
Wire Wire Line
	5650 2800 5650 2700
Connection ~ 5650 2700
$Comp
L Device:CP C1
U 1 1 5D9208E5
P 4350 3050
F 0 "C1" H 4468 3096 50  0000 L CNN
F 1 "4.7uF" H 4468 3005 50  0000 L CNN
F 2 "" H 4388 2900 50  0001 C CNN
F 3 "~" H 4350 3050 50  0001 C CNN
	1    4350 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Mini-DIN-4 J1
U 1 1 5D927187
P 4300 1800
F 0 "J1" H 4300 2167 50  0000 C CNN
F 1 "Connector to transducers arrays " H 4300 2076 50  0000 C CNN
F 2 "" H 4300 1800 50  0001 C CNN
F 3 "http://service.powerdynamics.com/ec/Catalog17/Section%2011.pdf" H 4300 1800 50  0001 C CNN
	1    4300 1800
	1    0    0    -1  
$EndComp
$Comp
L TeensyLev:Pololu_S18V20ALV U4
U 1 1 5D939936
P 3050 3050
F 0 "U4" H 2950 3550 50  0000 C CNN
F 1 "Pololu S18V20ALV" H 2950 3434 50  0000 C CNN
F 2 "" H 3050 3050 50  0001 C CNN
F 3 "" H 3050 3050 50  0001 C CNN
	1    3050 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT_US RV1
U 1 1 5D940C81
P 3050 4100
F 0 "RV1" V 2937 4100 50  0000 C CNN
F 1 "100K" V 2846 4100 50  0000 C CNN
F 2 "" H 3050 4100 50  0001 C CNN
F 3 "~" H 3050 4100 50  0001 C CNN
	1    3050 4100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7850 4100 3200 4100
Wire Wire Line
	3050 3950 3050 3350
$Comp
L power:GND #PWR?
U 1 1 5D94E83A
P 4350 3300
F 0 "#PWR?" H 4350 3050 50  0001 C CNN
F 1 "GND" H 4355 3127 50  0000 C CNN
F 2 "" H 4350 3300 50  0001 C CNN
F 3 "" H 4350 3300 50  0001 C CNN
	1    4350 3300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5D950C26
P 7550 2400
F 0 "#PWR?" H 7550 2150 50  0001 C CNN
F 1 "GND" H 7555 2227 50  0000 C CNN
F 2 "" H 7550 2400 50  0001 C CNN
F 3 "" H 7550 2400 50  0001 C CNN
	1    7550 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1700 4750 1700
Wire Wire Line
	4750 1700 4750 2500
Wire Wire Line
	4750 2600 4650 2600
Wire Wire Line
	4650 2600 4650 1800
Wire Wire Line
	4650 1800 4600 1800
Wire Wire Line
	4750 2700 4000 2700
Wire Wire Line
	4000 2700 4000 1800
Wire Wire Line
	4000 1700 3900 1700
Wire Wire Line
	3900 1700 3900 2800
Wire Wire Line
	3900 2800 4750 2800
Wire Wire Line
	4750 2900 4350 2900
Wire Wire Line
	4350 3200 4750 3200
Wire Wire Line
	4350 3200 4350 3300
Connection ~ 4350 3200
$Comp
L Connector:Jack-DC J1
U 1 1 5D958E99
P 1550 3000
F 0 "J1" H 1550 2800 50  0000 C CNN
F 1 "Jack-DC" H 1550 3200 50  0000 C CNN
F 2 "" H 1600 2960 50  0001 C CNN
F 3 "~" H 1600 2960 50  0001 C CNN
	1    1550 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2900 3600 2900
Connection ~ 4350 2900
$Comp
L Switch:SW_SPST SW1
U 1 1 5D95F1F3
P 2100 2900
F 0 "SW1" H 2100 3135 50  0000 C CNN
F 1 "Power switch" H 2100 3044 50  0000 C CNN
F 2 "" H 2100 2900 50  0001 C CNN
F 3 "~" H 2100 2900 50  0001 C CNN
	1    2100 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2900 1900 2900
Wire Wire Line
	2600 3100 2450 3100
$Comp
L power:GND #PWR?
U 1 1 5D973DE1
P 2450 3250
F 0 "#PWR?" H 2450 3000 50  0001 C CNN
F 1 "GND" H 2455 3077 50  0000 C CNN
F 2 "" H 2450 3250 50  0001 C CNN
F 3 "" H 2450 3250 50  0001 C CNN
	1    2450 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3250 2450 3100
Connection ~ 2450 3100
Wire Wire Line
	2450 3100 1850 3100
Wire Wire Line
	2300 2900 2450 2900
Wire Wire Line
	5600 2900 7850 2900
Wire Wire Line
	5600 3000 7850 3000
Wire Wire Line
	7850 3100 5600 3100
Wire Wire Line
	5600 3200 7850 3200
$Comp
L Device:R_US R1
U 1 1 5D98045E
P 7400 5050
F 0 "R1" H 7468 5096 50  0000 L CNN
F 1 "5K" H 7468 5005 50  0000 L CNN
F 2 "" V 7440 5040 50  0001 C CNN
F 3 "~" H 7400 5050 50  0001 C CNN
	1    7400 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R1
U 1 1 5D981BC8
P 7100 5050
F 0 "R1" H 7168 5096 50  0000 L CNN
F 1 "5K" H 7168 5005 50  0000 L CNN
F 2 "" V 7140 5040 50  0001 C CNN
F 3 "~" H 7100 5050 50  0001 C CNN
	1    7100 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4900 7400 4800
Wire Wire Line
	7400 4800 7850 4800
Wire Wire Line
	7100 4700 7100 4900
Wire Wire Line
	7100 5200 7100 5300
Wire Wire Line
	7100 5300 7250 5300
Wire Wire Line
	7400 5300 7400 5200
$Comp
L power:GND #PWR?
U 1 1 5D9873B6
P 7250 5500
F 0 "#PWR?" H 7250 5250 50  0001 C CNN
F 1 "GND" H 7255 5327 50  0000 C CNN
F 2 "" H 7250 5500 50  0001 C CNN
F 3 "" H 7250 5500 50  0001 C CNN
	1    7250 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 5300 7250 5500
Connection ~ 7250 5300
Wire Wire Line
	7250 5300 7400 5300
$Comp
L power:GND #PWR?
U 1 1 5D9B3AC5
P 5550 4800
F 0 "#PWR?" H 5550 4550 50  0001 C CNN
F 1 "GND" H 5555 4627 50  0000 C CNN
F 2 "" H 5550 4800 50  0001 C CNN
F 3 "" H 5550 4800 50  0001 C CNN
	1    5550 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 5D9B7803
P 10000 5300
F 0 "R2" H 10068 5346 50  0000 L CNN
F 1 "3.3K" H 10068 5255 50  0000 L CNN
F 2 "" V 10040 5290 50  0001 C CNN
F 3 "~" H 10000 5300 50  0001 C CNN
	1    10000 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R3
U 1 1 5D9B8ED6
P 10300 5000
F 0 "R3" V 10095 5000 50  0000 C CNN
F 1 "12K" V 10186 5000 50  0000 C CNN
F 2 "" V 10340 4990 50  0001 C CNN
F 3 "~" H 10300 5000 50  0001 C CNN
	1    10300 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	9850 5000 10000 5000
Wire Wire Line
	10000 5000 10000 5150
Connection ~ 10000 5000
Wire Wire Line
	10000 5000 10150 5000
Wire Wire Line
	10000 5450 10000 5500
Wire Wire Line
	10450 5000 10700 5000
Connection ~ 3600 2900
Wire Wire Line
	3600 2900 3300 2900
Wire Wire Line
	7850 2300 7550 2300
Wire Wire Line
	7550 2300 7550 2400
$Comp
L power:+3.3V #PWR?
U 1 1 5D9E087E
P 5550 4350
F 0 "#PWR?" H 5550 4200 50  0001 C CNN
F 1 "+3.3V" H 5565 4523 50  0000 C CNN
F 2 "" H 5550 4350 50  0001 C CNN
F 3 "" H 5550 4350 50  0001 C CNN
	1    5550 4350
	1    0    0    -1  
$EndComp
$Comp
L TeensyLev:Adafruit_3321_miniTFTwithJoy U3
U 1 1 5D9EACA7
P 6250 4500
F 0 "U3" H 6250 4000 50  0000 C CNN
F 1 "Adafruit 3321 miniTFTwithJoy" H 6350 4100 50  0000 C CNN
F 2 "" H 6300 4850 50  0001 C CNN
F 3 "" H 6300 4850 50  0001 C CNN
	1    6250 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 4700 7100 4700
Wire Wire Line
	7100 4700 6650 4700
Connection ~ 7100 4700
Wire Wire Line
	7400 4800 6650 4800
Connection ~ 7400 4800
Wire Wire Line
	7850 4200 7750 4200
Wire Wire Line
	7750 4200 7750 4600
Wire Wire Line
	7750 4600 6650 4600
Wire Wire Line
	7850 3500 7650 3500
Wire Wire Line
	7650 3500 7650 4500
Wire Wire Line
	7650 4500 6650 4500
Wire Wire Line
	7850 3400 7550 3400
Wire Wire Line
	7550 3400 7550 4400
Wire Wire Line
	7550 4400 6650 4400
Wire Wire Line
	7850 3300 7450 3300
Wire Wire Line
	7450 3300 7450 4300
Wire Wire Line
	7450 4300 6650 4300
Wire Wire Line
	3600 2900 3600 5950
Wire Wire Line
	3600 5950 10700 5950
Wire Wire Line
	10700 5950 10700 5000
$Comp
L power:GND #PWR?
U 1 1 5DA08117
P 10000 5500
F 0 "#PWR?" H 10000 5250 50  0001 C CNN
F 1 "GND" H 10005 5327 50  0000 C CNN
F 2 "" H 10000 5500 50  0001 C CNN
F 3 "" H 10000 5500 50  0001 C CNN
	1    10000 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 4700 5550 4700
Wire Wire Line
	5550 4700 5550 4800
Wire Wire Line
	5900 4400 5550 4400
Wire Wire Line
	5550 4400 5550 4350
$Comp
L power:+3.3V #PWR?
U 1 1 5DA16EB0
P 10250 2500
F 0 "#PWR?" H 10250 2350 50  0001 C CNN
F 1 "+3.3V" H 10265 2673 50  0000 C CNN
F 2 "" H 10250 2500 50  0001 C CNN
F 3 "" H 10250 2500 50  0001 C CNN
	1    10250 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 2500 10250 2700
Wire Wire Line
	10250 2700 9850 2700
$Comp
L power:GND #PWR?
U 1 1 5DA20B63
P 2600 1550
F 0 "#PWR?" H 2600 1300 50  0001 C CNN
F 1 "GND" H 2605 1377 50  0000 C CNN
F 2 "" H 2600 1550 50  0001 C CNN
F 3 "" H 2600 1550 50  0001 C CNN
	1    2600 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 2900 2450 1300
Wire Wire Line
	2450 1300 2650 1300
Connection ~ 2450 2900
Wire Wire Line
	2450 2900 2600 2900
Wire Wire Line
	3300 1250 10700 1250
Wire Wire Line
	10700 1250 10700 4600
Wire Wire Line
	10700 4600 9850 4600
Text Notes 750  3100 0    59   ~ 0
Power input\n5V-12V \n15W max
Text Notes 2700 4450 0    50   ~ 0
position around 45K\n
Wire Wire Line
	2650 1400 2600 1400
Wire Wire Line
	2600 1400 2600 1550
Text Notes 3100 3450 0    50   ~ 0
See hack-S18V20ALV.png\nfor details
$EndSCHEMATC
