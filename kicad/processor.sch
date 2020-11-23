EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 5
Title "Lucy"
Date "2020-11-23"
Rev "v1.0"
Comp "www.2-0.dk"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:C C11
U 1 1 5E07C9AE
P 4600 1400
F 0 "C11" H 4350 1350 50  0000 L CNN
F 1 "100n" H 4300 1450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4638 1250 50  0001 C CNN
F 3 "~" H 4600 1400 50  0001 C CNN
	1    4600 1400
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR028
U 1 1 5E07C9B5
P 4650 2700
F 0 "#PWR028" H 4650 2550 50  0001 C CNN
F 1 "+3V3" H 4665 2873 50  0000 C CNN
F 2 "" H 4650 2700 50  0001 C CNN
F 3 "" H 4650 2700 50  0001 C CNN
	1    4650 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 5E07C9C6
P 4650 4700
F 0 "#PWR029" H 4650 4450 50  0001 C CNN
F 1 "GND" H 4655 4527 50  0000 C CNN
F 2 "" H 4650 4700 50  0001 C CNN
F 3 "" H 4650 4700 50  0001 C CNN
	1    4650 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5E07C9E8
P 4600 1250
F 0 "#PWR030" H 4600 1000 50  0001 C CNN
F 1 "GND" H 4605 1077 50  0000 C CNN
F 2 "" H 4600 1250 50  0001 C CNN
F 3 "" H 4600 1250 50  0001 C CNN
	1    4600 1250
	-1   0    0    1   
$EndComp
Text GLabel 5150 3450 2    50   Output ~ 0
spi_sck
Text GLabel 5150 3550 2    50   Input ~ 0
spi_miso
Text GLabel 5150 3650 2    50   Output ~ 0
spi_mosi
Text GLabel 4050 4350 0    50   Output ~ 0
uart_tx
Text GLabel 5150 4250 2    50   BiDi ~ 0
swdio
Text GLabel 5150 4350 2    50   Input ~ 0
swclk
Text GLabel 5150 3150 2    50   BiDi ~ 0
mux_data
Text GLabel 4050 3850 0    50   Input ~ 0
adc_vbat
Text GLabel 5150 3950 2    50   BiDi ~ 0
radio_dio0
Text GLabel 5150 3850 2    50   BiDi ~ 0
radio_dio1
Text GLabel 5150 3750 2    50   BiDi ~ 0
radio_dio2
Text GLabel 4050 4050 0    50   Output ~ 0
radio_cs
Text GLabel 4050 4150 0    50   Output ~ 0
radio_reset
$Comp
L Device:C C?
U 1 1 5E07D9FE
P 5500 1400
AR Path="/5E07D9FE" Ref="C?"  Part="1" 
AR Path="/5E07BDE0/5E07D9FE" Ref="C13"  Part="1" 
F 0 "C13" H 5615 1446 50  0000 L CNN
F 1 "100n" H 5615 1355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5538 1250 50  0001 C CNN
F 3 "~" H 5500 1400 50  0001 C CNN
	1    5500 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5E07DA05
P 5950 1400
AR Path="/5E07DA05" Ref="C?"  Part="1" 
AR Path="/5E07BDE0/5E07DA05" Ref="C14"  Part="1" 
F 0 "C14" H 6065 1446 50  0000 L CNN
F 1 "100n" H 6065 1355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5988 1250 50  0001 C CNN
F 3 "~" H 5950 1400 50  0001 C CNN
	1    5950 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5E07DA13
P 5050 1400
AR Path="/5E07DA13" Ref="C?"  Part="1" 
AR Path="/5E07BDE0/5E07DA13" Ref="C12"  Part="1" 
F 0 "C12" H 5165 1446 50  0000 L CNN
F 1 "100n" H 5165 1355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5088 1250 50  0001 C CNN
F 3 "~" H 5050 1400 50  0001 C CNN
	1    5050 1400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E07DA20
P 5950 1550
AR Path="/5E07DA20" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E07DA20" Ref="#PWR036"  Part="1" 
F 0 "#PWR036" H 5950 1300 50  0001 C CNN
F 1 "GND" H 5955 1377 50  0000 C CNN
F 2 "" H 5950 1550 50  0001 C CNN
F 3 "" H 5950 1550 50  0001 C CNN
	1    5950 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E07DA26
P 5500 1550
AR Path="/5E07DA26" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E07DA26" Ref="#PWR034"  Part="1" 
F 0 "#PWR034" H 5500 1300 50  0001 C CNN
F 1 "GND" H 5505 1377 50  0000 C CNN
F 2 "" H 5500 1550 50  0001 C CNN
F 3 "" H 5500 1550 50  0001 C CNN
	1    5500 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E07DA2C
P 5050 1550
AR Path="/5E07DA2C" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E07DA2C" Ref="#PWR032"  Part="1" 
F 0 "#PWR032" H 5050 1300 50  0001 C CNN
F 1 "GND" H 5055 1377 50  0000 C CNN
F 2 "" H 5050 1550 50  0001 C CNN
F 3 "" H 5050 1550 50  0001 C CNN
	1    5050 1550
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E07DA38
P 5950 1250
AR Path="/5E07DA38" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E07DA38" Ref="#PWR035"  Part="1" 
F 0 "#PWR035" H 5950 1100 50  0001 C CNN
F 1 "+3V3" H 5965 1423 50  0000 C CNN
F 2 "" H 5950 1250 50  0001 C CNN
F 3 "" H 5950 1250 50  0001 C CNN
	1    5950 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E07DA3E
P 5500 1250
AR Path="/5E07DA3E" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E07DA3E" Ref="#PWR033"  Part="1" 
F 0 "#PWR033" H 5500 1100 50  0001 C CNN
F 1 "+3V3" H 5515 1423 50  0000 C CNN
F 2 "" H 5500 1250 50  0001 C CNN
F 3 "" H 5500 1250 50  0001 C CNN
	1    5500 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E07DA44
P 5050 1250
AR Path="/5E07DA44" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E07DA44" Ref="#PWR031"  Part="1" 
F 0 "#PWR031" H 5050 1100 50  0001 C CNN
F 1 "+3V3" H 5065 1423 50  0000 C CNN
F 2 "" H 5050 1250 50  0001 C CNN
F 3 "" H 5050 1250 50  0001 C CNN
	1    5050 1250
	1    0    0    -1  
$EndComp
Text GLabel 8250 3100 0    50   Input ~ 0
uart_tx
Text GLabel 8750 3000 2    50   BiDi ~ 0
swdio
Text GLabel 8750 3100 2    50   Output ~ 0
swclk
$Comp
L power:GND #PWR?
U 1 1 5E07DE54
P 8250 2900
AR Path="/5E07DE54" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E07DE54" Ref="#PWR039"  Part="1" 
F 0 "#PWR039" H 8250 2650 50  0001 C CNN
F 1 "GND" V 8255 2772 50  0000 R CNN
F 2 "" H 8250 2900 50  0001 C CNN
F 3 "" H 8250 2900 50  0001 C CNN
	1    8250 2900
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E07DE5A
P 8750 2900
AR Path="/5E07DE5A" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E07DE5A" Ref="#PWR043"  Part="1" 
F 0 "#PWR043" H 8750 2650 50  0001 C CNN
F 1 "GND" V 8755 2772 50  0000 R CNN
F 2 "" H 8750 2900 50  0001 C CNN
F 3 "" H 8750 2900 50  0001 C CNN
	1    8750 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5E07DE66
P 8750 2800
AR Path="/5E07DE66" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E07DE66" Ref="#PWR042"  Part="1" 
F 0 "#PWR042" H 8750 2650 50  0001 C CNN
F 1 "+3.3V" V 8765 2928 50  0000 L CNN
F 2 "" H 8750 2800 50  0001 C CNN
F 3 "" H 8750 2800 50  0001 C CNN
	1    8750 2800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E09E6C3
P 6750 3800
AR Path="/5E09E6C3" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E09E6C3" Ref="#PWR037"  Part="1" 
F 0 "#PWR037" H 6750 3550 50  0001 C CNN
F 1 "GND" V 6755 3672 50  0000 R CNN
F 2 "" H 6750 3800 50  0001 C CNN
F 3 "" H 6750 3800 50  0001 C CNN
	1    6750 3800
	0    1    1    0   
$EndComp
Text GLabel 7650 3700 2    50   Input ~ 0
hmi_red
Text GLabel 7650 3900 2    50   Input ~ 0
hmi_green
$Comp
L Device:R R?
U 1 1 5E09E6CB
P 7500 3700
AR Path="/5E09E6CB" Ref="R?"  Part="1" 
AR Path="/5E07BDE0/5E09E6CB" Ref="R4"  Part="1" 
F 0 "R4" V 7400 3700 50  0000 C CNN
F 1 "470" V 7500 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7430 3700 50  0001 C CNN
F 3 "~" H 7500 3700 50  0001 C CNN
	1    7500 3700
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 5E09E6D2
P 7500 3900
AR Path="/5E09E6D2" Ref="R?"  Part="1" 
AR Path="/5E07BDE0/5E09E6D2" Ref="R5"  Part="1" 
F 0 "R5" V 7400 3900 50  0000 C CNN
F 1 "470" V 7500 3900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7430 3900 50  0001 C CNN
F 3 "~" H 7500 3900 50  0001 C CNN
	1    7500 3900
	0    -1   1    0   
$EndComp
$Comp
L Device:LED_Dual_ACA D?
U 1 1 5E09E6D9
P 7050 3800
AR Path="/5E09E6D9" Ref="D?"  Part="1" 
AR Path="/5E07BDE0/5E09E6D9" Ref="D1"  Part="1" 
F 0 "D1" H 7050 4225 50  0000 C CNN
F 1 "red/green" H 7050 4134 50  0000 C CNN
F 2 "LED_THT:LED_D3.0mm-3" H 7050 3800 50  0001 C CNN
F 3 "~" H 7050 3800 50  0001 C CNN
	1    7050 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:Buzzer BZ?
U 1 1 5E09E6E0
P 6950 4450
AR Path="/5E09E6E0" Ref="BZ?"  Part="1" 
AR Path="/5E07BDE0/5E09E6E0" Ref="BZ1"  Part="1" 
F 0 "BZ1" H 7103 4479 50  0000 L CNN
F 1 "Buzzer" H 7103 4388 50  0000 L CNN
F 2 "Buzzer_Beeper:Buzzer_12x9.5RM7.6" V 6925 4550 50  0001 C CNN
F 3 "~" V 6925 4550 50  0001 C CNN
	1    6950 4450
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E09E6E7
P 7050 5100
AR Path="/5E09E6E7" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E09E6E7" Ref="#PWR041"  Part="1" 
F 0 "#PWR041" H 7050 4850 50  0001 C CNN
F 1 "GND" H 7055 4927 50  0000 C CNN
F 2 "" H 7050 5100 50  0001 C CNN
F 3 "" H 7050 5100 50  0001 C CNN
	1    7050 5100
	-1   0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E09E6F4
P 7050 4350
AR Path="/5E09E6F4" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5E09E6F4" Ref="#PWR040"  Part="1" 
F 0 "#PWR040" H 7050 4200 50  0001 C CNN
F 1 "+3V3" H 7065 4523 50  0000 C CNN
F 2 "" H 7050 4350 50  0001 C CNN
F 3 "" H 7050 4350 50  0001 C CNN
	1    7050 4350
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5E09E6FA
P 7550 4750
AR Path="/5E09E6FA" Ref="R?"  Part="1" 
AR Path="/5E07BDE0/5E09E6FA" Ref="R6"  Part="1" 
F 0 "R6" V 7450 4750 50  0000 C CNN
F 1 "1k" V 7550 4750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7480 4750 50  0001 C CNN
F 3 "~" H 7550 4750 50  0001 C CNN
	1    7550 4750
	0    -1   1    0   
$EndComp
Text GLabel 7700 4750 2    50   Input ~ 0
hmi_buzzer
Text GLabel 4600 1550 3    50   Output ~ 0
nrst
Text GLabel 4050 2950 0    50   BiDi ~ 0
nrst
Text GLabel 5150 2950 2    50   Output ~ 0
mux_a0
Text GLabel 5150 3250 2    50   Output ~ 0
mux_a1
Text GLabel 5150 3350 2    50   Output ~ 0
mux_a2
Text GLabel 5150 3050 2    50   Output ~ 0
mux_en
Text GLabel 5150 4450 2    50   Output ~ 0
pwr_sensor
Text GLabel 4050 3950 0    50   Output ~ 0
hmi_buzzer
Wire Wire Line
	4750 2750 4750 2700
Wire Wire Line
	4750 2700 4650 2700
Wire Wire Line
	4650 2700 4550 2700
Wire Wire Line
	4550 2700 4550 2750
Connection ~ 4650 2700
Wire Wire Line
	4650 2750 4650 2700
Wire Wire Line
	4550 4650 4550 4700
Wire Wire Line
	4550 4700 4650 4700
Wire Wire Line
	4650 4650 4650 4700
Connection ~ 4650 4700
$Comp
L lucy:AO3400A Q2
U 1 1 5E0B7E47
P 7150 4750
AR Path="/5E0B7E47" Ref="Q2"  Part="1" 
AR Path="/5E07BDE0/5E0B7E47" Ref="Q2"  Part="1" 
F 0 "Q2" H 7356 4796 50  0000 L CNN
F 1 "AO3400A" H 7356 4705 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7350 4850 50  0001 C CNN
F 3 "http://www.aosmd.com/pdfs/datasheet/AO3400A.pdf" H 7150 4750 50  0001 C CNN
	1    7150 4750
	-1   0    0    -1  
$EndComp
Text GLabel 4050 3150 0    50   Input ~ 0
boot0
$Comp
L Device:R R10
U 1 1 5E0C0BC4
P 4200 1400
F 0 "R10" H 4270 1446 50  0000 L CNN
F 1 "100k" H 4270 1355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4130 1400 50  0001 C CNN
F 3 "~" H 4200 1400 50  0001 C CNN
	1    4200 1400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5E0C0C12
P 4200 1250
F 0 "#PWR0101" H 4200 1000 50  0001 C CNN
F 1 "GND" H 4205 1077 50  0000 C CNN
F 2 "" H 4200 1250 50  0001 C CNN
F 3 "" H 4200 1250 50  0001 C CNN
	1    4200 1250
	-1   0    0    1   
$EndComp
Text GLabel 4200 1550 3    50   Output ~ 0
boot0
Text GLabel 8250 2800 0    50   Output ~ 0
boot0
$Comp
L MCU_ST_STM32L0:STM32L062K8Tx U3
U 1 1 5ED94B1E
P 4650 3650
F 0 "U3" H 4300 4600 50  0000 C CNN
F 1 "STM32L062K8Tx" V 4600 3650 50  0000 C CNN
F 2 "Package_QFP:LQFP-32_7x7mm_P0.8mm" H 4150 2750 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00108218.pdf" H 4650 3650 50  0001 C CNN
	1    4650 3650
	1    0    0    -1  
$EndComp
Text GLabel 5150 4150 2    50   Input ~ 0
usb_P
Text GLabel 5150 4050 2    50   Input ~ 0
usb_N
Text GLabel 2250 3800 2    50   Output ~ 0
usb_N
Text GLabel 2250 3900 2    50   Output ~ 0
usb_P
Wire Wire Line
	2150 3700 2150 3800
Wire Wire Line
	2150 3800 2250 3800
Wire Wire Line
	2150 3900 2250 3900
Wire Wire Line
	2150 3900 2150 4000
$Comp
L power:GND #PWR013
U 1 1 5EDD3D11
P 1550 4700
F 0 "#PWR013" H 1550 4450 50  0001 C CNN
F 1 "GND" H 1555 4527 50  0000 C CNN
F 2 "" H 1550 4700 50  0001 C CNN
F 3 "" H 1550 4700 50  0001 C CNN
	1    1550 4700
	1    0    0    -1  
$EndComp
Text GLabel 2250 3200 2    50   Output ~ 0
vusb
Wire Wire Line
	2250 3200 2150 3200
NoConn ~ 2150 4300
NoConn ~ 2150 4400
$Comp
L Device:R R14
U 1 1 5ED8632B
P 2350 3400
F 0 "R14" V 2250 3350 50  0000 L CNN
F 1 "5.1k" V 2350 3300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2280 3400 50  0001 C CNN
F 3 "~" H 2350 3400 50  0001 C CNN
	1    2350 3400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5ED89326
P 2600 3450
F 0 "#PWR014" H 2600 3200 50  0001 C CNN
F 1 "GND" V 2600 3250 50  0000 C CNN
F 2 "" H 2600 3450 50  0001 C CNN
F 3 "" H 2600 3450 50  0001 C CNN
	1    2600 3450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R15
U 1 1 5ED88C46
P 2350 3500
F 0 "R15" V 2450 3450 50  0000 L CNN
F 1 "5.1k" V 2350 3400 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2280 3500 50  0001 C CNN
F 3 "~" H 2350 3500 50  0001 C CNN
	1    2350 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 3400 2550 3400
Wire Wire Line
	2550 3400 2550 3450
Wire Wire Line
	2550 3450 2600 3450
Wire Wire Line
	2500 3500 2550 3500
Wire Wire Line
	2550 3500 2550 3450
Connection ~ 2550 3450
Wire Wire Line
	2150 3400 2200 3400
Wire Wire Line
	2150 3500 2200 3500
$Comp
L Power_Protection:SRV05-4 U8
U 1 1 5ED8FAA8
P 2450 5750
F 0 "U8" H 2700 6300 50  0000 C CNN
F 1 "SRV05-4" H 2750 6200 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 3150 5300 50  0001 C CNN
F 3 "http://www.onsemi.com/pub/Collateral/SRV05-4-D.PDF" H 2450 5750 50  0001 C CNN
	1    2450 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5ED91CB5
P 2450 6250
F 0 "#PWR023" H 2450 6000 50  0001 C CNN
F 1 "GND" H 2450 6100 50  0000 C CNN
F 2 "" H 2450 6250 50  0001 C CNN
F 3 "" H 2450 6250 50  0001 C CNN
	1    2450 6250
	1    0    0    -1  
$EndComp
Text GLabel 2450 5250 1    50   Output ~ 0
vusb
Text GLabel 1950 5650 0    50   Output ~ 0
usb_P
Text GLabel 1950 5850 0    50   Output ~ 0
usb_N
Text GLabel 2950 5650 2    50   Output ~ 0
usb_N
Text GLabel 2950 5850 2    50   Output ~ 0
usb_P
$Comp
L power:GND #PWR0107
U 1 1 5EE458B8
P 1250 4700
F 0 "#PWR0107" H 1250 4450 50  0001 C CNN
F 1 "GND" H 1255 4527 50  0000 C CNN
F 2 "" H 1250 4700 50  0001 C CNN
F 3 "" H 1250 4700 50  0001 C CNN
	1    1250 4700
	1    0    0    -1  
$EndComp
Text GLabel 8250 3000 0    50   Output ~ 0
uart_rx
Text GLabel 4050 4450 0    50   Input ~ 0
uart_rx
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J2
U 1 1 5EE22349
P 8450 3000
F 0 "J2" H 8500 3417 50  0000 C CNN
F 1 "Debug" H 8500 3326 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical" H 8450 3000 50  0001 C CNN
F 3 "~" H 8450 3000 50  0001 C CNN
	1    8450 3000
	1    0    0    -1  
$EndComp
Text GLabel 4050 3550 0    50   Output ~ 0
hmi_red
Text GLabel 4050 3650 0    50   Output ~ 0
hmi_green
$Comp
L power:GND #PWR?
U 1 1 5EE34B21
P 8750 3200
AR Path="/5EE34B21" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5EE34B21" Ref="#PWR0108"  Part="1" 
F 0 "#PWR0108" H 8750 2950 50  0001 C CNN
F 1 "GND" V 8755 3072 50  0000 R CNN
F 2 "" H 8750 3200 50  0001 C CNN
F 3 "" H 8750 3200 50  0001 C CNN
	1    8750 3200
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F0D50B8
P 7350 5100
AR Path="/5F0D50B8" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5F0D50B8" Ref="#PWR025"  Part="1" 
F 0 "#PWR025" H 7350 4850 50  0001 C CNN
F 1 "GND" H 7355 4927 50  0000 C CNN
F 2 "" H 7350 5100 50  0001 C CNN
F 3 "" H 7350 5100 50  0001 C CNN
	1    7350 5100
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5F0D5890
P 7350 4950
F 0 "R3" H 7420 4996 50  0000 L CNN
F 1 "10k" H 7420 4905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7280 4950 50  0001 C CNN
F 3 "~" H 7350 4950 50  0001 C CNN
	1    7350 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 4750 7400 4750
Wire Wire Line
	7050 4950 7050 5100
Wire Wire Line
	7350 4800 7350 4750
Connection ~ 2150 3900
Connection ~ 2150 3800
$Comp
L lucy:USB_C_Receptacle_USB2.0 J5
U 1 1 5EDCD952
P 1550 3800
F 0 "J5" H 1550 4650 50  0000 C CNN
F 1 "USB-C" H 1550 4550 50  0000 C CNN
F 2 "lucy:USB_C_Receptacle_HRO_TYPE-C-31-M-12" H 1700 3800 50  0001 C CNN
F 3 "https://www.usb.org/sites/default/files/documents/usb_type-c.zip" H 1700 3800 50  0001 C CNN
	1    1550 3800
	1    0    0    -1  
$EndComp
Text GLabel 4050 4250 0    50   BiDi ~ 0
mux_data
$Comp
L power:GND #PWR?
U 1 1 5FBD0670
P 8250 3200
AR Path="/5FBD0670" Ref="#PWR?"  Part="1" 
AR Path="/5E07BDE0/5FBD0670" Ref="#PWR02"  Part="1" 
F 0 "#PWR02" H 8250 2950 50  0001 C CNN
F 1 "GND" V 8255 3072 50  0000 R CNN
F 2 "" H 8250 3200 50  0001 C CNN
F 3 "" H 8250 3200 50  0001 C CNN
	1    8250 3200
	0    1    1    0   
$EndComp
$EndSCHEMATC
