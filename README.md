# stm32f446_inmp441
[WIP] stm32f446 inmp441 i2s and sai test code backup, only for fun, not good

## About sreenshot jpg (serial output)
* Use Arduino IDE->Serial Plotter to see curve chart  

## usb-microphone, I2S  
```
���ڲ��ýӣ�uart2�ڵ�RX��TX����Ӧ���Ҳ��TX/D1,RX/D0����Ҫ��������ϣ�Ĭ�ϱ����ӵ�st-link�����⴮����  
usart2, TX, PA2(not need)==>TX/D1  
usart2, RX, PA3(not need)==>RX/D0  

INMP441  
SCK,WS,LR  
[===]xxxxxxRRRRR  
SD,VDD,GND  

INMP441<->STM32F446  
SCK(left top 1)<->I2S1_CK, PB3 (D3, right 2 bottom 4)  
SD(right top 1)<->I2S1_SD, PA7 (D11, right 1 top 7)  
WS(left top 2)<->I2S1_WS, PA4 (left 2 bottom 4)  
VDD(right top 2)<->3.3(left 3 top 4)  
L/R(left top 3)<->GND  
GND(right top 3)<->GND(left 3 top 7)  
```

## nucleo-f446-ei-kws, SAI    
```
���ڲ��ýӣ�uart2�ڵ�RX��TX����Ӧ���Ҳ��TX/D1,RX/D0����Ҫ��������ϣ�Ĭ�ϱ����ӵ�st-link�����⴮����  
usart2, TX, PA2(not need)==>TX/D1  
usart2, RX, PA3(not need)==>RX/D0  

INMP441  
SCK,WS,LR  
[===]xxxxxxRRRRR  
SD,VDD,GND  

INMP441<->STM32F446  
SCK(left top 1)<->SAI1_SCK_B, PB12 (right 3 top 8)  
SD(right top 1)<->SAI1_SD_B, PA9 (D8, right 1 top 10)  
WS(left top 2)<->SAI1_FS_B, PB9 (right 2 top 3)  
VDD(right top 2)<->3.3(left 3 top 4)  
L/R(left top 3)<->GND  
GND(right top 3)<->GND(left 3 top 7)  
```
