# Energy-Saving-System
Embedded System Project
![](Aspose.Words.a38cbf35-d0cb-4315-b13c-3e464c093027.001.png)

1. **Objective**  

The purpose of the Energy Consumption Detector is to prevent unnecessary waste and contribute to sustainability by keeping the amount of energy used at the optimum level in every environment where light and heat energy is used (home, workplace, institutions, etc.). Since it is thought that the system's self-management of energy may pose a danger, it is aimed to inform the user, offer suggestions, and make calculations about energy saving with a mobile application. In addition, when extraordinary values are reached in terms of heat and light, it is expected to detect an emergency and notify the user by giving a red alarm.   

2. **Features** 
1) Real-time Energy Monitoring: The system has sensors that allow you to monitor energy consumption and energy usage data in various areas.   
1) Calculation of Energy Usage: The system provides algorithms to calculate energy usage accurately and the data is provided by the sensors. Total energy consumption will be calculated by using a current sensor and warnings will be displayed according to the values coming from the temperature sensor and light sensor.  
- With a smart algorithm, the system will be able to predict and suggest better results.  
3) It makes some predictions on the energy consumption for different time intervals based on given data and makes suggestions accordingly.  
3) With a Bluetooth module, the system is integrated with mobile applications. So, the users will be able to monitor/see the energy usage remotely. In the mobile application, additional features such as energy-saving tips can be added.  

![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 002](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/80b28cd0-0316-47d2-8f88-0a6d3794542e)
]

***Figure1: Design of the project*** 

3. **Components** 
- **Sensors  with ADC** 

In order to get the current temperature, LM35 temperature sensor is being used with Analog to Digital Converter. The temperature value is read from the sensor with the aid of ADC pin and then the obtained value is converted to Celsius with the aid of following equations: 

- ((*  ×  *3*~~.~~*3*))/*4095* 
  - ×  *100*  

In addition to LM35, a light sensor ( LDR) is used to detect the light intensity in the environment. Similar to LM35, an analog value is read from the sensor and with the aid of an ADC, it is converted to digital. 

- **Display** 

A 16\*2 LCD display is used for monitoring the values read from the sensors after the ADC conversion. 

- **GPIO** 

We used some GPIO output pins to connect leds to the circuit which will be turned on in case of emergencies or after some specific changes.  

- **Motor Driver and Lamp** 

Light intensity of the environment is measured with a lamp connected to the system. When the lamp uses a volt above 4.5, it will be considered as an emergency and the red LED will warn. The ADC value of 4.5 volt is calculated by:  

*4*~~.~~*5 1*

× *212*  + = *5585*~~.~~*5* 

*3*~~.~~*3 2*

- **Communication** 

The data obtained from the sensors are transferred to the virtual terminal via USART communication. 

- **DMA** 

DMA is used in conjunction with SPI (Serial Peripheral Interface) to send temperature values from a peripheral device to memory. 

- **Timer** 

We used TIM1 for doing the DMA operation on a regular basis. 

- **Interrupt** 
- We used interrupts while making predictions in order to do other jobs during the process.  
- **Mobile Application** 
- We developed a mobile application to control our system and enhance user experience. The user will be able to connect the system and on/off the system.  
- **Bluetooth Module** 

To make the project more helpful, we created a mobile application and connected it  to our project. We put in a Bluetooth module. We may connect to the system from our phone and turn it on and off with the help of this Bluetooth module. The Bluetooth module we used is Bluetooth HC-05. In order to use this module in proteus, we added its library in Keil library.  

- **Smart Algorithm** 

To make the project more smart, we implemented a linear regression. With the help of the algorithm we will be able to make predictions about the future energy consumption. Additionally, this predicted value is sent to the mobile application.  
![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 003](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/c0cddbe7-6715-4c15-9097-5ba240e94a86)

**III.  Implementation** 

STM32 microcontroller, including ADC reading, LCD display, smart algorithm for prediction, Bluetooth module for mobile application connection, and LED control. It combines multiple peripheral configurations and functions to achieve the desired functionality. Simulation is used on proteus. 

**CubeMX** 

![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 004](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/f5c0748f-b3a7-4a7e-bc76-d8b8f433f67e)

Leds:** PB13, PB14 and PB15 pins are set as GPIO Output pins for led connections.**  
![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 005](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/e12d1a24-ed35-4368-aba4-0af06cfd0117)

UART: The UART baud rate is set to 9600 and the word length is set to 8 bits. PA9 and PA10 pins are cross connected to UART1 because of the asynchronous mode and PA1 and PA2 pins are cross-connected to UART2. UART1 is used for sensor data while UART2 is used for the Bluetooth module. 
![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 006](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/5548e56f-cf90-41a2-86fa-209a56bd72a5)

![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 007](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/1420c402-172a-42e1-a825-7719f06d252d)

LCD Display: ADC is activated in NVIC settings in CubeMX. It has a printing value between 1-100. We monitor the sensor data via a 7-segment LCD display. 
![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 008](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/0b365d37-3ff4-4884-9315-06a3aeafac46)

**Keil MDK** 

The code includes several header files, such as "main.h," "stdio.h," "LCD.h," "stdlib.h," and "math.h," which provide necessary definitions, function prototypes, and standard library functions. We used an external library for defining LCD functions.We also used different types of private variables, such as flag, readValue1, readValue2, readValue3, sConfigPrivate, and temperature to store ADC readings, configuration settings, and temperature values. The code declares several function prototypes, including SystemClock\_Config() for system clock configuration, and various functions for initializing peripherals such as GPIO, ADC, UART, SPI, and timers. Some of the functions are explained below: 

*Read\_ADC():* This function reads the ADC values from three different channels (0, 1, and 2) using the HAL library. It configures the ADC channel, starts the ADC conversion, waits for the conversion to complete, and stores the ADC readings in the respective variables. 

*show\_status():* This function calculates the temperature value based on the ADC readings and compares it with predefined thresholds. It sets or resets GPIO pins accordingly to indicate temperature conditions. 

*lcd\_screen():* This function displays the ADC values on an LCD screen. It initializes the LCD, prepares the ADC readings as strings, and uses the lcd\_print() function to display the values on specific lines of the LCD. 

*smart\_algorithm():* This function implements a simple linear regression algorithm to predict a value based on given input arrays x and y. It calculates the slope (m) and intercept (b) of the regression line and stores them in corresponding variables. 

The program enters an infinite loop where it repeatedly reads ADC values, displays them on the LCD, performs the smart algorithm for prediction, and controls LEDs based on certain conditions. It also includes UART communication for receiving commands from an external device. 

**Proteus** 

After completing the configuration on CubeMX and writing the project function in KeilMDK, we started to work on our Proteus design. First of all, we added our sensors and made the ADC connections and all other necessary connections and we also added a virtual terminal to see if we can get the data correctly. At this point we added a motor to detect the current passing through it in order to calculate the energy consumption. After that, we added the leds which work synchronously with the sensors. Then we added the Bluetooth module which allows us to communicate with our application and connect it to the terminal to see the output appropriately. We also used SPI for DMA which we aim to pass the temperature value. 

5. **Challenges** 

One of the biggest problems that we had was sending the data correctly to the LCD display and we also were not able to send our temperature data to the virtual terminal via UART appropriately. We were able to solve the issue after switching to Proteus 8.11. Another issue we had was Bluetooth connection between mobile application and microcontroller. We needed to add serial ports separately. We managed to get the ADC values as an interrupt in Keil, but unfortunately, the callback function did not work in proteus. We changed versions for this, we used different methods, and we got help from TAs, but unfortunately, we could not solve this problem.

6. **Project Screenshots** 
![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 009](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/c0743687-8df5-4f4a-a1e2-6b13522de163)

![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 010](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/2cbd4b09-94fa-4d28-8fe1-5589c780ee63)


7. **Mobile Application Screenshot** 
![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 011](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/35cf50ff-bf3b-4c0d-afa7-4e5e933345b9)


*The user will be able to connect to the system by pressing the active* 
![Aspose Words a38cbf35-d0cb-4315-b13c-3e464c093027 012](https://github.com/ZehraMogulkoc/Energy-Saving-System/assets/87859856/cd1baf34-b79f-466d-8fea-79ae60a12568)


*After connecting to the system, the user can choose to turn on or turn off the* 

**VIII. Demo Video** 

[https://drive.google.com/drive/folders/14t-](https://drive.google.com/drive/folders/14t-qJGQDgcDZOGvBD3nRhErHlM4KXlHO?usp=sharing)              [qJGQDgcDZOGvBD3nRhErHlM4KXlHO?usp=sharing ](https://drive.google.com/drive/folders/14t-qJGQDgcDZOGvBD3nRhErHlM4KXlHO?usp=sharing) 

**IX. Features table** 



|Module/Feature  |Types  |Used types  |
| - | - | - |
|GPIO  |<p>- Digital Output   </p><p>- Digital Input   </p><p>- Other  </p>|●  Digital Output: Push/Pull Alert Led  |
|Communication  |<p>●UART   </p><p>- SPI   </p><p>- I2C  </p><p>- CAN, Others  </p><p>- Using multiple devices at the  same  communication bus  </p>|<p>- UART: </p><p>- Interrupt for inputs and Bluetooth (HC-05) </p><p>- SPI:  </p><p>- Sending the temperature value from peripheral to memory </p>|
|Watchdog Timer  |||
|Interactivity (Leds,  buttons, switches,  touch etc.)  ||<p>- Leds </p><p>- LCD Screen </p><p>- Virtual Terminal </p><p>- Bluetooth Module </p>|
|Using Sensors  |<p>- Single  </p><p>- Few  </p><p>- Many or advanced one  </p>|<p>- ACS712- Current sensor  </p><p>- LM35- Temperature sensor  </p><p>- LDR – Light sensor  </p>|
|Actuators  |<p>- Motors  </p><p>- ..  </p>|●  DC Motor for air conditioning |
|Timers  |<p>- Systick  </p><p>- Advanced-basic Timers  </p><p>- RTC alarm  </p>|` `Timer for DMA |



|Usage of polling  ||` `Taking ADC values |
| - | :- | - |
|Usage of Interrupts  |<p>- No interrupt   </p><p>- Single  </p><p>- Few  </p><p>- Many  with  different priorities  </p>|●  Few interrupts |
|Error handling  |<p>- No error handling   </p><p>- Few  </p><p>- Full  </p>|●  Few |
|Advanced Things  that no code is  provided during the  course such as extra  DAC, CAN etc.  |extra  ||
|Power saving  |Sleep - standby - wakeup  ||
|DMA  ||●  Sending  the  temperature  value from peripheral to memory |
|Ethernet-internet-  Wi-Fi  |||
|Writing own driver  library for a peripheral  |||
|Bluetooth  ||X  |
|PCB  |●  External  electronics||
||<p>design   </p><p>- Using a different board   </p><p>- Using MCU unit on your own  design  without  the development board  </p>||
|Usage of advanced  tools e.g., Matlab,  CubeAI etc.  (Matlab code  should run on  MCU)  |||


||||
| :- | :- | :- |
|Real time OS  |||
|IoT  |<p>Making it work with:  </p><p>- Node-red, Blynk etc.   </p><p>- Mobile device interaction</p><p>- Time  series  database (InfluxDB)  </p>|●  Mobile Device interaction |

