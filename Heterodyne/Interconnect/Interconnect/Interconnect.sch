EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "BoSL Velocity Interconnect"
Date "2020-08-04"
Rev "rev 0.2"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR021
U 1 1 5EF998A7
P 9350 3650
F 0 "#PWR021" H 9350 3400 50  0001 C CNN
F 1 "GND" V 9355 3522 50  0000 R CNN
F 2 "" H 9350 3650 50  0001 C CNN
F 3 "" H 9350 3650 50  0001 C CNN
	1    9350 3650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5EF99D8D
P 4450 3700
F 0 "#PWR04" H 4450 3450 50  0001 C CNN
F 1 "GND" V 4455 3572 50  0000 R CNN
F 2 "" H 4450 3700 50  0001 C CNN
F 3 "" H 4450 3700 50  0001 C CNN
	1    4450 3700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR020
U 1 1 5EF9A90E
P 9350 3550
F 0 "#PWR020" H 9350 3400 50  0001 C CNN
F 1 "+5V" V 9365 3678 50  0000 L CNN
F 2 "" H 9350 3550 50  0001 C CNN
F 3 "" H 9350 3550 50  0001 C CNN
	1    9350 3550
	0    -1   -1   0   
$EndComp
$Comp
L power:VPP #PWR03
U 1 1 5EF9AE8E
P 4450 3600
F 0 "#PWR03" H 4450 3450 50  0001 C CNN
F 1 "VPP" V 4465 3728 50  0000 L CNN
F 2 "" H 4450 3600 50  0001 C CNN
F 3 "" H 4450 3600 50  0001 C CNN
	1    4450 3600
	0    1    1    0   
$EndComp
Text Notes 9650 4000 0    50   ~ 0
TX
Text Notes 9650 3900 0    50   ~ 0
RX
Text Notes 4050 3950 0    50   ~ 0
RX
Text Notes 4050 4050 0    50   ~ 0
TX
Text Notes 9650 3800 0    50   ~ 0
RST
Text Notes 4050 3850 0    50   ~ 0
RST
Text Notes 4050 4150 0    50   ~ 0
WKE
Text Notes 4050 3550 0    50   ~ 0
EN
Text Notes 9650 4100 0    50   ~ 0
WKE
$Comp
L Interconnect-rescue:SN74LV1T34DBVR-project U2
U 1 1 5EFB5562
P 6350 2350
F 0 "U2" H 6594 2096 50  0000 L CNN
F 1 "SN74LV1T34DBVR" H 6594 2005 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 6350 2350 50  0001 C CNN
F 3 "" H 6350 2350 50  0001 C CNN
	1    6350 2350
	1    0    0    -1  
$EndComp
$Comp
L Interconnect-rescue:SN74LV1T34DBVR-project U4
U 1 1 5EFB63FF
P 6350 4550
F 0 "U4" H 6106 4204 50  0000 R CNN
F 1 "SN74LV1T34DBVR" H 6106 4295 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 6350 4550 50  0001 C CNN
F 3 "" H 6350 4550 50  0001 C CNN
	1    6350 4550
	-1   0    0    1   
$EndComp
$Comp
L Interconnect-rescue:SN74LV1T34DBVR-project U5
U 1 1 5EFB6900
P 6350 4750
F 0 "U5" H 6594 4496 50  0000 L CNN
F 1 "SN74LV1T34DBVR" H 6594 4405 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 6350 4750 50  0001 C CNN
F 3 "" H 6350 4750 50  0001 C CNN
	1    6350 4750
	1    0    0    -1  
$EndComp
$Comp
L Interconnect-rescue:SN74LV1T34DBVR-project U3
U 1 1 5EFB5DB4
P 6350 3150
F 0 "U3" H 6594 2896 50  0000 L CNN
F 1 "SN74LV1T34DBVR" H 6594 2805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 6350 3150 50  0001 C CNN
F 3 "" H 6350 3150 50  0001 C CNN
	1    6350 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 3800 6000 3800
Wire Wire Line
	6000 3800 6000 2650
Wire Wire Line
	6000 2650 6150 2650
Wire Wire Line
	4450 3900 6100 3900
Wire Wire Line
	6100 3900 6100 3450
Wire Wire Line
	6100 3450 6150 3450
Wire Wire Line
	4450 4000 6100 4000
Wire Wire Line
	6100 4000 6100 4250
Wire Wire Line
	6100 4250 6150 4250
Wire Wire Line
	4450 4100 6000 4100
Wire Wire Line
	6000 4100 6000 5050
Wire Wire Line
	6000 5050 6150 5050
Wire Wire Line
	6550 3450 7300 3450
Wire Wire Line
	7300 3450 7300 3850
Wire Wire Line
	7300 3850 9350 3850
Wire Wire Line
	6550 4250 7300 4250
Wire Wire Line
	7300 4250 7300 3950
Wire Wire Line
	7300 3950 9350 3950
Wire Wire Line
	6550 5050 7400 5050
Wire Wire Line
	7400 5050 7400 4050
Wire Wire Line
	7400 4050 9350 4050
Wire Wire Line
	9350 3750 7400 3750
Wire Wire Line
	7400 3750 7400 2650
$Comp
L power:GND #PWR014
U 1 1 5EFBAFDE
P 6300 3900
F 0 "#PWR014" H 6300 3650 50  0001 C CNN
F 1 "GND" H 6305 3727 50  0000 C CNN
F 2 "" H 6300 3900 50  0001 C CNN
F 3 "" H 6300 3900 50  0001 C CNN
	1    6300 3900
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5EFBB50F
P 6400 5400
F 0 "#PWR019" H 6400 5150 50  0001 C CNN
F 1 "GND" H 6405 5227 50  0000 C CNN
F 2 "" H 6400 5400 50  0001 C CNN
F 3 "" H 6400 5400 50  0001 C CNN
	1    6400 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5EFBB7EC
P 6400 3800
F 0 "#PWR017" H 6400 3550 50  0001 C CNN
F 1 "GND" H 6405 3627 50  0000 C CNN
F 2 "" H 6400 3800 50  0001 C CNN
F 3 "" H 6400 3800 50  0001 C CNN
	1    6400 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5EFBBBF6
P 6400 3000
F 0 "#PWR016" H 6400 2750 50  0001 C CNN
F 1 "GND" H 6405 2827 50  0000 C CNN
F 2 "" H 6400 3000 50  0001 C CNN
F 3 "" H 6400 3000 50  0001 C CNN
	1    6400 3000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR012
U 1 1 5EFBC114
P 6300 2300
F 0 "#PWR012" H 6300 2150 50  0001 C CNN
F 1 "+5V" H 6315 2473 50  0000 C CNN
F 2 "" H 6300 2300 50  0001 C CNN
F 3 "" H 6300 2300 50  0001 C CNN
	1    6300 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR013
U 1 1 5EFBCBA1
P 6300 3100
F 0 "#PWR013" H 6300 2950 50  0001 C CNN
F 1 "+5V" H 6315 3273 50  0000 C CNN
F 2 "" H 6300 3100 50  0001 C CNN
F 3 "" H 6300 3100 50  0001 C CNN
	1    6300 3100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR015
U 1 1 5EFBD10E
P 6300 4700
F 0 "#PWR015" H 6300 4550 50  0001 C CNN
F 1 "+5V" H 6315 4873 50  0000 C CNN
F 2 "" H 6300 4700 50  0001 C CNN
F 3 "" H 6300 4700 50  0001 C CNN
	1    6300 4700
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR018
U 1 1 5EFBD884
P 6400 4600
F 0 "#PWR018" H 6400 4450 50  0001 C CNN
F 1 "+3V3" H 6415 4773 50  0000 C CNN
F 2 "" H 6400 4600 50  0001 C CNN
F 3 "" H 6400 4600 50  0001 C CNN
	1    6400 4600
	-1   0    0    1   
$EndComp
$Comp
L Regulator_Switching:TPS61222DCK U1
U 1 1 5EFD134E
P 3750 4900
F 0 "U1" H 3750 5267 50  0000 C CNN
F 1 "TPS61222DCK" H 3750 5176 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:Texas_R-PDSO-G6" H 3750 4100 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps61220.pdf" H 3750 4750 50  0001 C CNN
	1    3750 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 4800 4250 4800
Wire Wire Line
	4250 4800 4250 5000
Wire Wire Line
	4250 5000 4150 5000
Wire Wire Line
	4250 4800 4400 4800
Connection ~ 4250 4800
$Comp
L power:GND #PWR011
U 1 1 5EFD3D7F
P 4400 5100
F 0 "#PWR011" H 4400 4850 50  0001 C CNN
F 1 "GND" H 4405 4927 50  0000 C CNN
F 2 "" H 4400 5100 50  0001 C CNN
F 3 "" H 4400 5100 50  0001 C CNN
	1    4400 5100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5EFD41CE
P 3750 5200
F 0 "#PWR06" H 3750 4950 50  0001 C CNN
F 1 "GND" H 3755 5027 50  0000 C CNN
F 2 "" H 3750 5200 50  0001 C CNN
F 3 "" H 3750 5200 50  0001 C CNN
	1    3750 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5EFD44A3
P 2800 5050
F 0 "C1" H 2915 5096 50  0000 L CNN
F 1 "10 μF" H 2400 5050 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2838 4900 50  0001 C CNN
F 3 "https://www.digikey.com/product-detail/en/kemet/C0805C106K8PACTU/399-4925-1-ND/1090920" H 2800 5050 50  0001 C CNN
	1    2800 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4900 3350 4900
$Comp
L pspice:INDUCTOR L1
U 1 1 5EFD50CD
P 3050 4800
F 0 "L1" H 3050 5015 50  0000 C CNN
F 1 "NR3015T4R7M" H 3050 4924 50  0000 C CNN
F 2 "Inductor_SMD:L_Taiyo-Yuden_NR-30xx" H 3050 4800 50  0001 C CNN
F 3 "https://www.digikey.com/product-detail/en/taiyo-yuden/NR3015T4R7M/587-1649-1-ND/1008264" H 3050 4800 50  0001 C CNN
	1    3050 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4800 2800 4900
Connection ~ 2800 4900
Wire Wire Line
	3300 4800 3350 4800
$Comp
L power:GND #PWR01
U 1 1 5EFD6121
P 2800 5200
F 0 "#PWR01" H 2800 4950 50  0001 C CNN
F 1 "GND" H 2805 5027 50  0000 C CNN
F 2 "" H 2800 5200 50  0001 C CNN
F 3 "" H 2800 5200 50  0001 C CNN
	1    2800 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5EFD6712
P 3150 5150
F 0 "R1" H 3220 5196 50  0000 L CNN
F 1 "10 kΩ" H 3220 5105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3080 5150 50  0001 C CNN
F 3 "~" H 3150 5150 50  0001 C CNN
	1    3150 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 5000 3350 5000
$Comp
L power:GND #PWR02
U 1 1 5EFD77F4
P 3150 5300
F 0 "#PWR02" H 3150 5050 50  0001 C CNN
F 1 "GND" H 3155 5127 50  0000 C CNN
F 2 "" H 3150 5300 50  0001 C CNN
F 3 "" H 3150 5300 50  0001 C CNN
	1    3150 5300
	1    0    0    -1  
$EndComp
Connection ~ 3350 5000
Text GLabel 3350 5850 3    50   Input ~ 0
EN
Text GLabel 4750 4950 0    50   Input ~ 0
EN
$Comp
L power:+5V #PWR010
U 1 1 5EFD8BA9
P 5550 4800
F 0 "#PWR010" H 5550 4650 50  0001 C CNN
F 1 "+5V" V 5565 4928 50  0000 L CNN
F 2 "" H 5550 4800 50  0001 C CNN
F 3 "" H 5550 4800 50  0001 C CNN
	1    5550 4800
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5EFD9494
P 4400 4950
F 0 "C2" H 4500 4950 50  0000 L CNN
F 1 "10 μF" H 4500 4850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4438 4800 50  0001 C CNN
F 3 "https://www.digikey.com/product-detail/en/kemet/C0805C106K8PACTU/399-4925-1-ND/1090920" H 4400 4950 50  0001 C CNN
	1    4400 4950
	1    0    0    -1  
$EndComp
Connection ~ 4400 4800
$Comp
L power:+5V #PWR08
U 1 1 5EFDB5DE
P 4850 2750
F 0 "#PWR08" H 4850 2600 50  0001 C CNN
F 1 "+5V" H 4865 2923 50  0000 C CNN
F 2 "" H 4850 2750 50  0001 C CNN
F 3 "" H 4850 2750 50  0001 C CNN
	1    4850 2750
	0    -1   -1   0   
$EndComp
$Comp
L power:+3V3 #PWR09
U 1 1 5EFDE04E
P 5300 2900
F 0 "#PWR09" H 5300 2750 50  0001 C CNN
F 1 "+3V3" V 5315 3028 50  0000 L CNN
F 2 "" H 5300 2900 50  0001 C CNN
F 3 "" H 5300 2900 50  0001 C CNN
	1    5300 2900
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5EFE28BE
P 5000 2750
F 0 "R3" H 5070 2796 50  0000 L CNN
F 1 "680 Ω" H 5070 2705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4930 2750 50  0001 C CNN
F 3 "~" H 5000 2750 50  0001 C CNN
	1    5000 2750
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 5EFE2C27
P 5000 3050
F 0 "R2" H 5150 3000 50  0000 R CNN
F 1 "1.5 kΩ" H 5300 3100 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4930 3050 50  0001 C CNN
F 3 "~" H 5000 3050 50  0001 C CNN
	1    5000 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5EFE55EB
P 4850 3050
F 0 "#PWR07" H 4850 2800 50  0001 C CNN
F 1 "GND" H 4855 2877 50  0000 C CNN
F 2 "" H 4850 3050 50  0001 C CNN
F 3 "" H 4850 3050 50  0001 C CNN
	1    4850 3050
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 5EFEBE4C
P 3500 5700
F 0 "JP1" H 3500 5905 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 3500 5814 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" H 3500 5700 50  0001 C CNN
F 3 "~" H 3500 5700 50  0001 C CNN
	1    3500 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 5850 3350 5700
Wire Wire Line
	3350 5000 3350 5700
Connection ~ 3350 5700
$Comp
L power:VPP #PWR05
U 1 1 5EFEDB97
P 3650 5700
F 0 "#PWR05" H 3650 5550 50  0001 C CNN
F 1 "VPP" V 3665 5828 50  0000 L CNN
F 2 "" H 3650 5700 50  0001 C CNN
F 3 "" H 3650 5700 50  0001 C CNN
	1    3650 5700
	0    1    1    0   
$EndComp
$Comp
L power:VPP #PWR0101
U 1 1 5F0120A7
P 2800 4900
F 0 "#PWR0101" H 2800 4750 50  0001 C CNN
F 1 "VPP" V 2815 5027 50  0000 L CNN
F 2 "" H 2800 4900 50  0001 C CNN
F 3 "" H 2800 4900 50  0001 C CNN
	1    2800 4900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6550 2650 7400 2650
Wire Wire Line
	5150 3050 5300 3050
Wire Wire Line
	5300 3050 5300 2900
Wire Wire Line
	5300 2900 5300 2750
Wire Wire Line
	5300 2750 5150 2750
Connection ~ 5300 2900
$Comp
L Connector_Generic:Conn_01x07 J1
U 1 1 5F28EA64
P 4250 3800
F 0 "J1" H 4168 4317 50  0000 C CNN
F 1 "Conn_01x07" H 4168 4226 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x07_P2.54mm_Horizontal" H 4250 3800 50  0001 C CNN
F 3 "~" H 4250 3800 50  0001 C CNN
	1    4250 3800
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J2
U 1 1 5F28F645
P 9550 3850
F 0 "J2" H 9550 4150 50  0000 L CNN
F 1 "Conn_01x06" H 9550 3400 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Horizontal" H 9550 3850 50  0001 C CNN
F 3 "~" H 9550 3850 50  0001 C CNN
	1    9550 3850
	1    0    0    1   
$EndComp
$Comp
L Device:C C3
U 1 1 5F298E98
P 5500 4950
F 0 "C3" H 5615 4996 50  0000 L CNN
F 1 "1 μF" H 5650 4900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5538 4800 50  0001 C CNN
F 3 "https://www.digikey.com/product-detail/en/kemet/C0805C106K8PACTU/399-4925-1-ND/1090920" H 5500 4950 50  0001 C CNN
	1    5500 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 4800 4750 4800
Wire Wire Line
	5400 4800 5500 4800
Connection ~ 5500 4800
Wire Wire Line
	5500 4800 5550 4800
$Comp
L power:GND #PWR024
U 1 1 5F29C138
P 5500 5100
F 0 "#PWR024" H 5500 4850 50  0001 C CNN
F 1 "GND" H 5505 4927 50  0000 C CNN
F 2 "" H 5500 5100 50  0001 C CNN
F 3 "" H 5500 5100 50  0001 C CNN
	1    5500 5100
	1    0    0    -1  
$EndComp
$Comp
L Interconnect-rescue:MIC94090 U6
U 1 1 5F2A1973
P 4850 4800
F 0 "U6" H 5075 5025 50  0000 C CNN
F 1 "MIC94090" H 5075 4934 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6" H 4850 4800 50  0001 C CNN
F 3 "" H 4850 4800 50  0001 C CNN
	1    4850 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5F2A714D
P 5050 5100
F 0 "#PWR022" H 5050 4850 50  0001 C CNN
F 1 "GND" H 5055 4927 50  0000 C CNN
F 2 "" H 5050 5100 50  0001 C CNN
F 3 "" H 5050 5100 50  0001 C CNN
	1    5050 5100
	1    0    0    -1  
$EndComp
Text GLabel 4450 3500 2    50   Input ~ 0
EN
$EndSCHEMATC
