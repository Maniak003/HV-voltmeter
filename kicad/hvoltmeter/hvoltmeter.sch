EESchema Schematic File Version 4
EELAYER 30 0
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
L Device:C C7
U 1 1 632CFD1F
P 4750 5350
F 0 "C7" H 4800 5600 50  0000 L CNN
F 1 "200" H 4800 5500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4788 5200 50  0001 C CNN
F 3 "~" H 4750 5350 50  0001 C CNN
	1    4750 5350
	1    0    0    -1  
$EndComp
$Comp
L hvoltmeter-rescue:USB_B_Micro-Connector-My_Library J1
U 1 1 632D2EFA
P 2650 3000
F 0 "J1" H 2707 3375 50  0000 C CNN
F 1 "USB_B_Micro-Connector" H 2707 3376 50  0001 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex_47346-0001" H 2800 2950 50  0001 C CNN
F 3 "" H 2800 2950 50  0001 C CNN
	1    2650 3000
	1    0    0    -1  
$EndComp
$Comp
L hvoltmeter-rescue:STM32G031F8-My_Library-My_Library D2
U 1 1 632D3B69
P 2900 4550
F 0 "D2" H 2900 5265 50  0000 C CNN
F 1 "STM32G031F8" H 2900 5174 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 2900 4200 50  0001 C CNN
F 3 "" H 2900 4200 50  0001 C CNN
	1    2900 4550
	1    0    0    -1  
$EndComp
$Comp
L Battery_Management:LTC4054ES5-4.2 U1
U 1 1 632D482D
P 3900 3100
F 0 "U1" H 4000 2900 50  0000 L CNN
F 1 "LTC4054ES5-4.2" H 4344 3005 50  0001 L CNN
F 2 "Package_TO_SOT_SMD:TSOT-23-5" H 3900 2600 50  0001 C CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/405442xf.pdf" H 3900 3000 50  0001 C CNN
	1    3900 3100
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:AD8603 U2
U 1 1 632D7971
P 4200 5200
F 0 "U2" H 4200 5200 50  0000 C CNN
F 1 "AD8541" H 4300 5050 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TSOT-23-5" H 4200 5200 50  0001 C CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/AD8603_8607_8609.pdf" H 4200 5400 50  0001 C CNN
	1    4200 5200
	-1   0    0    -1  
$EndComp
$Comp
L hvoltmeter-rescue:Conn-My_Library J4
U 1 1 632DFC6A
P 4500 3400
F 0 "J4" V 4396 3448 50  0001 L CNN
F 1 "Conn" V 4650 3400 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 4550 3250 50  0001 C CNN
F 3 "~" H 4500 3400 50  0001 C CNN
	1    4500 3400
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_DPDT_x2 SW1
U 1 1 632E16DF
P 4700 3100
F 0 "SW1" H 4700 3385 50  0001 C CNN
F 1 "SW_DPDT_x2" H 4700 3294 50  0001 C CNN
F 2 "My-library:MySwithc" H 4700 3100 50  0001 C CNN
F 3 "~" H 4700 3100 50  0001 C CNN
	1    4700 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 632E3196
P 2450 4500
F 0 "#PWR02" H 2450 4250 50  0001 C CNN
F 1 "GND" H 2455 4327 50  0001 C CNN
F 2 "" H 2450 4500 50  0001 C CNN
F 3 "" H 2450 4500 50  0001 C CNN
	1    2450 4500
	0    1    1    0   
$EndComp
$Comp
L Regulator_Linear:MCP1700-2802E_SOT23 U3
U 1 1 632E4954
P 5300 3000
F 0 "U3" H 5300 3242 50  0000 C CNN
F 1 "MCP1700" H 5300 3151 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5300 3225 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 5300 3000 50  0001 C CNN
	1    5300 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 632E8119
P 3200 2950
F 0 "D1" V 3200 3150 50  0000 R CNN
F 1 "LED" V 3100 3200 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3200 2950 50  0001 C CNN
F 3 "~" H 3200 2950 50  0001 C CNN
	1    3200 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 632F152A
P 3350 3100
F 0 "R1" V 3150 3150 50  0000 C CNN
F 1 "1k" V 3250 3150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3280 3100 50  0001 C CNN
F 3 "~" H 3350 3100 50  0001 C CNN
	1    3350 3100
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 632F1DA9
P 3500 3550
F 0 "R2" H 3570 3596 50  0000 L CNN
F 1 "20k" H 3570 3505 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3430 3550 50  0001 C CNN
F 3 "~" H 3500 3550 50  0001 C CNN
	1    3500 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 632F2421
P 3900 3700
F 0 "#PWR04" H 3900 3450 50  0001 C CNN
F 1 "GND" H 3905 3527 50  0001 C CNN
F 2 "" H 3900 3700 50  0001 C CNN
F 3 "" H 3900 3700 50  0001 C CNN
	1    3900 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3200 3500 3400
Connection ~ 3500 3700
Wire Wire Line
	3500 3700 3900 3700
Wire Wire Line
	3900 3700 3900 3500
Connection ~ 3900 3700
$Comp
L hvoltmeter-rescue:Conn-My_Library J5
U 1 1 632FDD5C
P 4500 3600
F 0 "J5" V 4488 3552 50  0001 R CNN
F 1 "Conn" V 4397 3552 50  0001 R CNN
F 2 "My-library:SMD-CONN" H 4550 3450 50  0001 C CNN
F 3 "~" H 4500 3600 50  0001 C CNN
	1    4500 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3900 3700 4300 3700
$Comp
L Device:C C3
U 1 1 632FF60D
P 4300 3550
F 0 "C3" H 4050 3600 50  0000 L CNN
F 1 "10uF" H 4000 3500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4338 3400 50  0001 C CNN
F 3 "~" H 4300 3550 50  0001 C CNN
	1    4300 3550
	1    0    0    -1  
$EndComp
Connection ~ 4300 3700
Wire Wire Line
	4300 3400 4300 3100
Connection ~ 4300 3100
$Comp
L Device:C C2
U 1 1 633029CF
P 3100 3550
F 0 "C2" H 2850 3600 50  0000 L CNN
F 1 "10uF" H 2800 3500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3138 3400 50  0001 C CNN
F 3 "~" H 3100 3550 50  0001 C CNN
	1    3100 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 3400 2550 3700
Wire Wire Line
	2650 3400 2550 3400
Connection ~ 2550 3400
$Comp
L Device:C C6
U 1 1 6330F5A3
P 5000 3550
F 0 "C6" H 5115 3596 50  0000 L CNN
F 1 "0.1" H 5115 3505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5038 3400 50  0001 C CNN
F 3 "~" H 5000 3550 50  0001 C CNN
	1    5000 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 3000 5000 3000
Wire Wire Line
	5000 3400 5000 3000
Connection ~ 5000 3000
Connection ~ 5000 3700
$Comp
L Device:C C8
U 1 1 63314299
P 5600 3550
F 0 "C8" H 5715 3596 50  0000 L CNN
F 1 "0.1" H 5715 3505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5638 3400 50  0001 C CNN
F 3 "~" H 5600 3550 50  0001 C CNN
	1    5600 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 6332A8E0
P 2300 4750
F 0 "C1" H 2050 4750 50  0000 L CNN
F 1 "0.1" H 2050 4650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2338 4600 50  0001 C CNN
F 3 "~" H 2300 4750 50  0001 C CNN
	1    2300 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 6332B0D0
P 2300 4900
F 0 "#PWR03" H 2300 4650 50  0001 C CNN
F 1 "GND" H 2305 4727 50  0001 C CNN
F 2 "" H 2300 4900 50  0001 C CNN
F 3 "" H 2300 4900 50  0001 C CNN
	1    2300 4900
	1    0    0    -1  
$EndComp
$Comp
L power:+2V8 #PWR01
U 1 1 6332B718
P 2450 4400
F 0 "#PWR01" H 2450 4250 50  0001 C CNN
F 1 "+2V8" V 2465 4528 50  0000 L CNN
F 2 "" H 2450 4400 50  0001 C CNN
F 3 "" H 2450 4400 50  0001 C CNN
	1    2450 4400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3900 5200 3900 5550
$Comp
L power:GND #PWR08
U 1 1 63335BF3
P 4750 5500
F 0 "#PWR08" H 4750 5250 50  0001 C CNN
F 1 "GND" H 4755 5327 50  0001 C CNN
F 2 "" H 4750 5500 50  0001 C CNN
F 3 "" H 4750 5500 50  0001 C CNN
	1    4750 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 633361E5
P 4600 5350
F 0 "R5" H 4550 5800 50  0000 L CNN
F 1 "10M" H 4500 5700 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4530 5350 50  0001 C CNN
F 3 "~" H 4600 5350 50  0001 C CNN
	1    4600 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 63336864
P 4750 4850
F 0 "R6" H 4850 4900 50  0000 L CNN
F 1 "5G" H 4850 4800 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 4680 4850 50  0001 C CNN
F 3 "~" H 4750 4850 50  0001 C CNN
	1    4750 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 5500 4600 5500
$Comp
L power:+2V8 #PWR07
U 1 1 633492D6
P 4300 4900
F 0 "#PWR07" H 4300 4750 50  0001 C CNN
F 1 "+2V8" H 4150 4900 50  0000 C CNN
F 2 "" H 4300 4900 50  0001 C CNN
F 3 "" H 4300 4900 50  0001 C CNN
	1    4300 4900
	1    0    0    -1  
$EndComp
$Comp
L hvoltmeter-rescue:Conn-My_Library J6
U 1 1 6334DD34
P 4750 4600
F 0 "J6" V 4646 4648 50  0001 L CNN
F 1 "Conn" V 4900 4600 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 4800 4450 50  0001 C CNN
F 3 "~" H 4750 4600 50  0001 C CNN
	1    4750 4600
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C4
U 1 1 6334E4F1
P 3900 4100
F 0 "C4" H 3900 4200 50  0000 L CNN
F 1 "0.1" H 3900 4000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3938 3950 50  0001 C CNN
F 3 "~" H 3900 4100 50  0001 C CNN
	1    3900 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 6334EDF5
P 4100 4100
F 0 "C5" H 4100 4200 50  0000 L CNN
F 1 "0.1" H 4100 4000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4138 3950 50  0001 C CNN
F 3 "~" H 4100 4100 50  0001 C CNN
	1    4100 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3950 4100 3950
$Comp
L power:+2V8 #PWR09
U 1 1 63350E2C
P 4100 3950
F 0 "#PWR09" H 4100 3800 50  0001 C CNN
F 1 "+2V8" H 4115 4123 50  0000 C CNN
F 2 "" H 4100 3950 50  0001 C CNN
F 3 "" H 4100 3950 50  0001 C CNN
	1    4100 3950
	1    0    0    -1  
$EndComp
Connection ~ 4100 3950
$Comp
L power:GND #PWR010
U 1 1 6335150E
P 4100 4250
F 0 "#PWR010" H 4100 4000 50  0001 C CNN
F 1 "GND" H 4105 4077 50  0001 C CNN
F 2 "" H 4100 4250 50  0001 C CNN
F 3 "" H 4100 4250 50  0001 C CNN
	1    4100 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4250 4100 4250
Connection ~ 4100 4250
$Comp
L Device:R R3
U 1 1 6335A741
P 3700 4600
F 0 "R3" V 3900 4550 50  0000 L CNN
F 1 "10k" V 3800 4550 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3630 4600 50  0001 C CNN
F 3 "~" H 3700 4600 50  0001 C CNN
	1    3700 4600
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 6335AF8C
P 3700 4800
F 0 "R4" V 3600 4750 50  0000 L CNN
F 1 "20k" V 3500 4750 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3630 4800 50  0001 C CNN
F 3 "~" H 3700 4800 50  0001 C CNN
	1    3700 4800
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 6335B576
P 3850 4800
F 0 "#PWR06" H 3850 4550 50  0001 C CNN
F 1 "GND" H 3855 4627 50  0001 C CNN
F 2 "" H 3850 4800 50  0001 C CNN
F 3 "" H 3850 4800 50  0001 C CNN
	1    3850 4800
	0    -1   -1   0   
$EndComp
$Comp
L power:+BATT #PWR011
U 1 1 6335BFDF
P 5000 3000
F 0 "#PWR011" H 5000 2850 50  0001 C CNN
F 1 "+BATT" H 4850 3100 50  0000 C CNN
F 2 "" H 5000 3000 50  0001 C CNN
F 3 "" H 5000 3000 50  0001 C CNN
	1    5000 3000
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR05
U 1 1 6335C7CF
P 3850 4600
F 0 "#PWR05" H 3850 4450 50  0001 C CNN
F 1 "+BATT" V 3800 4850 50  0000 C CNN
F 2 "" H 3850 4600 50  0001 C CNN
F 3 "" H 3850 4600 50  0001 C CNN
	1    3850 4600
	0    1    1    0   
$EndComp
NoConn ~ 2450 4200
NoConn ~ 2450 4300
NoConn ~ 2450 4700
NoConn ~ 2450 4800
NoConn ~ 2450 4900
NoConn ~ 2450 5000
NoConn ~ 3350 4100
NoConn ~ 3350 4700
NoConn ~ 3350 4800
$Comp
L hvoltmeter-rescue:Conn-My_Library J3
U 1 1 633707DA
P 3450 4300
F 0 "J3" V 3346 4348 50  0001 L CNN
F 1 "SWD" H 3450 4300 50  0000 L CNN
F 2 "My-library:SMD-CONN" H 3500 4150 50  0001 C CNN
F 3 "~" H 3450 4300 50  0001 C CNN
	1    3450 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 3700 5300 3700
Wire Wire Line
	5300 3300 5300 3700
Connection ~ 5300 3700
Wire Wire Line
	5300 3700 5600 3700
Wire Wire Line
	5600 3000 5600 3400
$Comp
L power:+2V8 #PWR012
U 1 1 632E3A37
P 5600 3000
F 0 "#PWR012" H 5600 2850 50  0001 C CNN
F 1 "+2V8" V 5615 3128 50  0000 L CNN
F 2 "" H 5600 3000 50  0001 C CNN
F 3 "" H 5600 3000 50  0001 C CNN
	1    5600 3000
	0    1    1    0   
$EndComp
Connection ~ 5600 3000
$Comp
L hvoltmeter-rescue:Conn-My_Library J2
U 1 1 6336FFB2
P 3450 4200
F 0 "J2" V 3346 4248 50  0001 L CNN
F 1 "SWC" H 3450 4200 50  0000 L CNN
F 2 "My-library:SMD-CONN" H 3500 4050 50  0001 C CNN
F 3 "~" H 3450 4200 50  0001 C CNN
	1    3450 4200
	1    0    0    -1  
$EndComp
$Comp
L hvoltmeter-rescue:OLED049-My_Library D3
U 1 1 6342BAD8
P 5650 4300
F 0 "D3" H 6000 3550 50  0000 L CNN
F 1 "OLED049" H 5900 3450 50  0000 L CNN
F 2 "My-library:OLED049" H 5650 4300 50  0001 C CNN
F 3 "" H 5650 4300 50  0001 C CNN
	1    5650 4300
	1    0    0    -1  
$EndComp
Text GLabel 3350 4400 2    50   Input ~ 0
SDA
Text GLabel 3350 4500 2    50   Input ~ 0
SCL
Text GLabel 5450 4900 0    50   Input ~ 0
SDA
Text GLabel 5450 4800 0    50   Input ~ 0
SCL
$Comp
L power:GND #PWR014
U 1 1 634318F9
P 5450 4500
F 0 "#PWR014" H 5450 4250 50  0001 C CNN
F 1 "GND" H 5455 4327 50  0001 C CNN
F 2 "" H 5450 4500 50  0001 C CNN
F 3 "" H 5450 4500 50  0001 C CNN
	1    5450 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	2300 4600 2450 4600
$Comp
L Device:C C11
U 1 1 6345A8A7
P 5300 3900
F 0 "C11" V 5500 3650 50  0000 C CNN
F 1 "1uF" V 5350 3650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5338 3750 50  0001 C CNN
F 3 "~" H 5300 3900 50  0001 C CNN
	1    5300 3900
	0    1    1    0   
$EndComp
$Comp
L Device:C C12
U 1 1 6345B55B
P 5300 4100
F 0 "C12" V 5050 3850 50  0000 C CNN
F 1 "1uF" V 5400 3850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5338 3950 50  0001 C CNN
F 3 "~" H 5300 4100 50  0001 C CNN
	1    5300 4100
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 4000 5150 4000
Wire Wire Line
	5150 4000 5150 3900
Wire Wire Line
	5450 4200 5150 4200
Wire Wire Line
	5150 4200 5150 4100
$Comp
L Device:C C10
U 1 1 6345E0BF
P 5250 5350
F 0 "C10" H 5250 5600 50  0000 R CNN
F 1 "2.2uF" H 5250 5700 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5288 5200 50  0001 C CNN
F 3 "~" H 5250 5350 50  0001 C CNN
	1    5250 5350
	-1   0    0    1   
$EndComp
$Comp
L Device:C C13
U 1 1 6345E99C
P 5450 5350
F 0 "C13" H 5350 5600 50  0000 R CNN
F 1 "4.7uF" H 5350 5700 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5488 5200 50  0001 C CNN
F 3 "~" H 5450 5350 50  0001 C CNN
	1    5450 5350
	-1   0    0    1   
$EndComp
$Comp
L Device:R R7
U 1 1 63469488
P 5100 5350
F 0 "R7" H 4900 5400 50  0000 L CNN
F 1 "560k" H 4850 5300 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5030 5350 50  0001 C CNN
F 3 "~" H 5100 5350 50  0001 C CNN
	1    5100 5350
	1    0    0    -1  
$EndComp
NoConn ~ 5450 4400
Connection ~ 5100 5500
Wire Wire Line
	5100 5500 5250 5500
$Comp
L power:GND #PWR013
U 1 1 6346D257
P 5100 5500
F 0 "#PWR013" H 5100 5250 50  0001 C CNN
F 1 "GND" H 5105 5327 50  0001 C CNN
F 2 "" H 5100 5500 50  0001 C CNN
F 3 "" H 5100 5500 50  0001 C CNN
	1    5100 5500
	1    0    0    -1  
$EndComp
Connection ~ 5250 5500
Wire Wire Line
	5250 5500 5450 5500
Wire Wire Line
	5450 5100 5250 5100
Wire Wire Line
	5250 5100 5250 5200
Wire Wire Line
	5450 5000 5100 5000
Wire Wire Line
	5100 5000 5100 5200
$Comp
L Device:C C9
U 1 1 6348D6C1
P 4300 4100
F 0 "C9" H 4300 4200 50  0000 L CNN
F 1 "0.1" H 4300 4000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4338 3950 50  0001 C CNN
F 3 "~" H 4300 4100 50  0001 C CNN
	1    4300 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3950 4300 3950
Wire Wire Line
	4100 4250 4300 4250
Wire Wire Line
	3350 4600 3500 4600
Wire Wire Line
	3500 4600 3500 4800
Text GLabel 2450 4100 0    50   Input ~ 0
Ain
Text GLabel 3850 5200 0    50   Input ~ 0
Ain
Wire Wire Line
	3850 5200 3900 5200
Connection ~ 3900 5200
NoConn ~ 3350 4900
NoConn ~ 3350 5000
Connection ~ 3200 2800
Wire Wire Line
	3200 2800 3900 2800
$Comp
L power:+2V8 #PWR016
U 1 1 63592993
P 5000 4300
F 0 "#PWR016" H 5000 4150 50  0001 C CNN
F 1 "+2V8" V 5015 4428 50  0000 L CNN
F 2 "" H 5000 4300 50  0001 C CNN
F 3 "" H 5000 4300 50  0001 C CNN
	1    5000 4300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 6359D97D
P 5000 4450
F 0 "R8" H 4800 4400 50  0000 L CNN
F 1 "10k" H 4800 4500 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4930 4450 50  0001 C CNN
F 3 "~" H 5000 4450 50  0001 C CNN
	1    5000 4450
	-1   0    0    1   
$EndComp
Connection ~ 4600 5500
Wire Wire Line
	4500 5100 4600 5100
Wire Wire Line
	4600 5200 4600 5100
Wire Wire Line
	4600 5500 4750 5500
Connection ~ 4600 5100
Wire Wire Line
	4600 5100 4750 5100
Wire Wire Line
	4750 5000 4750 5100
Connection ~ 4750 5100
Wire Wire Line
	4750 5100 4750 5200
Connection ~ 4750 5500
Wire Wire Line
	4500 5550 3900 5550
Wire Wire Line
	4500 5550 4500 5300
Wire Wire Line
	5450 4300 5250 4300
Wire Wire Line
	3100 3700 3500 3700
Wire Wire Line
	3100 3400 3100 2800
Wire Wire Line
	3100 2800 3200 2800
Wire Wire Line
	2550 3700 3100 3700
Connection ~ 3100 3700
Wire Wire Line
	2950 2800 3100 2800
Connection ~ 3100 2800
Wire Wire Line
	5250 4300 5250 4600
Wire Wire Line
	5250 4600 5450 4600
Connection ~ 5250 4300
Wire Wire Line
	5250 4300 5000 4300
Connection ~ 5000 4300
Wire Wire Line
	5000 4600 5000 4700
Wire Wire Line
	5000 4700 5450 4700
Wire Wire Line
	3500 4600 3550 4600
Connection ~ 3500 4600
Wire Wire Line
	3500 4800 3550 4800
Wire Wire Line
	4500 3100 4500 3300
Wire Wire Line
	4500 3300 4900 3300
Wire Wire Line
	4900 3300 4900 3200
Connection ~ 4500 3100
Connection ~ 4500 3300
Connection ~ 4500 3700
Wire Wire Line
	4500 3700 5000 3700
Wire Wire Line
	4300 3100 4500 3100
Wire Wire Line
	4300 3700 4500 3700
$EndSCHEMATC
