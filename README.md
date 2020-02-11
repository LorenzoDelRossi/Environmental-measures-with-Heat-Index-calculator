# Environmental Measures with Heat Index Calculator

## Scope of the project

In this project we aimed to learn the capabilities of the STM32 Discovery Kit IOT node by ST Electronics using FreeRTOS, a market leading real-time operating system for microcontrollers and small microprocessors.

In particular we used the Capacitive digital sensor for relative humidity and temperature (HTS221)  and the 260-1260 hPa absolute digital output barometer (LPS22HB) to get the environmental measures, and then we used temperature and relative humidity to estimate the Heat Index.

Each of these measures were computed using a different task, giving us the chance to test and learn task creation and synchronisation using the freeRTOS libraries.

At the end of the project, we were able to synchronise different tasks with different priorities, and create an asynchronous task that manipulated the results of two of these tasks (temperature and humidity).

Here is the flow chart of our code:

![flowchart](/images/flow-chart.png)

## How we achieved it

Our first aim was to use different tasks, one for each sensor, get the values and then print all of them together.

#### Event Flags

To achieve the synchronization we used CMSIS-RTOS2's event flags management functions. These functions allows a task to wait and check for certain bits, called event flags, and to start running once the event flag bits are set.

The main task (in our case, the print task) waits forever for the other tasks' bits to be set. Each of the other tasks reads its respective sensor value and sets a different bit. Since we have three values to be read, the main task will wait three bits to be set, and only then it prints to the output terminal our humidity, temperature and pressure values.

```c
flags = osEventFlagsWait(evt_id, BIT_ALL, osFlagsWaitAll, osWaitForever);
```

The first task in our routine, the temperature, sets the first bit. 

```
osEventFlagsSet(evt_id, BIT_TEMP);
```

Inside the second task the osEventFlagsWait function waits for the first bit to be set, and only then sets the second bit. 

```c
flags = osEventFlagsWait(evt_id, BIT_TEMP, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
pres_value = BSP_PSENSOR_ReadPressure();
...
osEventFlagsSet(evt_id, BIT_PRES);
```

The third and last task will then wait for both the first two bits to be set. 

```c
flags = osEventFlagsWait(evt_id, BIT_TEMP | BIT_PRES, osFlagsWaitAll | osFlagsNoClear, osWaitForever);
hum_value = BSP_HSENSOR_ReadHumidity();
...
osEventFlagsSet(evt_id, BIT_HUM);
```

Once the three bits are set, our main task starts running and prints the three computed values.

```c
snprintf(str_stamp,100,"HUMIDITY = %d.%02d %%\r\nTEMPERATURE = %d.%02d \xB0""C\r\nPRESSURE = %d.%02d hPa\r\n\r\n", Inthum1, Inthum2, Inttemp1, Inttemp2, Intpres1, Intpres2);
```

In order to prove that the event flags are indeed working properly we put an osDelay of 1 second inside the StartPrintTask (which prints all the readings) and an osDelay of 3 seconds inside the three environmental measures tasks. If the event flags were not working properly we would see the print task starting without waiting for the others to set their bits.

The next step to realize our project was inserting an aperiodic task, activated by the user button, without disrupting our main routine.

#### Message Queues

We took advantage of CMSIS-RTOS2 support for message queues, and created a queue with a capacity of only 1 message.

```c
 ButtonQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &ButtonQueue_attributes);
```

When the button is pressed, its callback puts an empty message inside the queue (signal passing).

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13){ // If I just pressed the User Button
	osMessageQueuePut(ButtonQueueHandle, &msg, 0U, 0U);}
}
```

The asynchronous task (in our case the Heat Index calculator) waits indefinitely for a message in the queue, then it starts computing and outputting the Heat Index together with the other readings, and lastly it toggles the LED2.

```c
status = osMessageQueueGet(ButtonQueueHandle, &msg, NULL, osWaitForever);   // wait for message
	      if (status == osOK) {
	    	  ...
	    	  snprintf(str_hi,100,"Temperature = %d.%02d \xB0""C, Humidity = %d.%02d %%, Heat Index = %d.%02d\r\n\r\n", Inttemp1, Inttemp2, Inthum1, Inthum2, Inthi1, Inthi2);
	    	  HAL_UART_Transmit(&huart1,( uint8_t * )str_hi,sizeof(str_hi),1000);
	    	  HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
	      }
```

This solution is efficient since we have a single signal queue, so even if the button is repeatedly pressed we don't have synchronization problems. Furthermore, it's a lightweight solution since the message is represented by an 8 bit unsigned integer.

## Final Thoughts

This project aimed to explore some of the tools available using the CMSIS-RTOS2 operating system, using STM32CubeIDE as a front-end.

From the start, it was remarkable that only using delays we were able to achieve a working project with different tasks running without interfering with each other.

Although using delays can result in a viable solution for simpler projects, this OS shows its capabilities with the Event Flags management functions, making it easy and fast to synchronize different tasks while maintaining data consistency and integrity. Even the asynchronous task was efficiently handled by the operating system using a simple signal queue without consequences on the main routine.

With these considerations and the connectivity capabilities of the STM32L475 using Bluetooth, NFC, and Wi-Fi, the Discovery Kit - IoT Node makes for a great start in developing embedded Real Time applications.
