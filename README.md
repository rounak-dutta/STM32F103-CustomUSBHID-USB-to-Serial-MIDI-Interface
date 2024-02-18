# STM32F103-CustomUSBHID-USB-to-Serial-MIDI-Interface
In this project a very basic USB to Serial MIDI Interface is built with STM32F103C8T6 board, with USB Custom-HID Middleware implementation in STM CubeIDE environment.


**Notes:** Before going through the project and/ or trying to implement the project, I would request the viewer to go through **the following drawbacks or limitations of the project,** as follows:

  i. I currently do not own any MIDI instruments which support serial-MIDI, so I tested by connecting the uC TX-pin to the RX-pin with a piece of wire (i.e., loopback test), and sent MIDI data from DAW to the uC and back into the DAW to a virtual instrument, and it is able to relay most of the MIDI data. But in faster paced sections of the MIDI-song, there are few lost-notes/ lost-MIDI-data, thus the USB 2 Serial conversion is not 100% lossless, atleast in the loop-back test.
  
  ii. I have spent a considerable amount of time trying to debug the lost-message issue, I have tried Interrupt UART, DMA UART (as in current implementation), but nothing could get me 100% transmission and reception. If I place a USB send-report immedietly after the USB data receive (i.e., bypassing the UART), then I don't get the issue, thus the data-loss has something to do with the UART interface. If anyone finds a solution to this issue please feel free to leave a comment in this project or in the YouTube video associated with this.
  
  iii. In this implementation, I have only considered the 3-byte MIDI messages, which generally covers the Note-On, Note-Off, Sustain Pedal, Pitch-Bend, etc., i.e., the most common types of MIDI-messages. The 2-bytes, 1-byte and SysEx MIDI messages hadling is not implemented. 



**The steps to implement Custom-HID USB-MIDI for STM32 in CubeIDE, is as follows:**
1. Launch the STM CubeIDE and select uC stm32f103c8t6.
2. In the GUI interface, enable external ceramic oscillator, enable serial-wire debug.
3. Select USART2 (or any other USART, as per availability) in asynchronous mode with BAUD rate of 31250 (which is the serial MIDI baud-rate).
4. For the USART add DMA for both TX and RX, and set priority to HIGH for both, the enable the NVIC global interrupt for the USART.
5. Enable USB in Device mode and select Custom HID in the middleware section.
6. In the clock setup, set the MUX to PLL mode and select 72MHz frequncy and press enter.
7. Click on save and generate code.

8. In Top --> USB_DEVICE --> Target --> usbd_conf.h
	change: 
		USBD_CUSTOMHID_OUTREPORT_BUF_SIZE 	2 to 4.

9. In Top --> Middlewares --> ST --> STM32_USB_Device_Library --> Class --> CustomHID --> Inc --> usbd_customhid.h
	change:
		CUSTOM_HID_EPIN_SIZE			0x02 to 0x04
		CUSTOM_HID_EPOUT_SIZE			0x02 to 0x04
	In Function Prototype: _USBD_CUSTOM_HID_Itf: int8_t (* OutEvent) (uint8_t event_idx, uint8_t state); to int8_t (* OutEvent) (uint8_t* );

10. In Top --> Middlewares --> ST --> STM32_USB_Device_Library --> Class --> CustomHID --> Src --> usbd_customhid.c
	In function USBD_CUSTOM_HID_DataOut, change: 
		((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->OutEvent(hhid->Report_buf[0],hhid->Report_buf[1]); 
		to  ((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->OutEvent(hhid->Report_buf);

	In Function USBD_CUSTOM_HID_EP0_RxReady,change: 
		((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->OutEvent(hhid->Report_buf[0],hhid->Report_buf[1]); 
		to      ((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->OutEvent(hhid->Report_buf);

	In Arrays: USBD_CUSTOM_HID_CfgFSDesc, USBD_CUSTOM_HID_CfgHSDesc & USBD_CUSTOM_HID_OtherSpeedCfgDesc, replace the existing Configuration-descriptors
	with the descriptor in the "USB_MIDI_CFG_Descriptors.txt" file obtained form the USB HID MIDI official doc.
	Also, in the arrays, remove the Array-size variables and keep it [].
	
	In "USBD_ClassTypeDef  USBD_CUSTOM_HID", replace: 	
					USBD_CUSTOM_HID_GetHSCfgDesc,
  					USBD_CUSTOM_HID_GetFSCfgDesc,
  					USBD_CUSTOM_HID_GetOtherSpeedCfgDesc,

			to		USBD_CUSTOM_HID_GetFSCfgDesc,
  					USBD_CUSTOM_HID_GetFSCfgDesc,
  					USBD_CUSTOM_HID_GetFSCfgDesc,

11. In Top --> USB_DEVICE --> App --> usbd_custom_hid_if.c
	In the prototype, change:
	 	static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state); 
		to	static int8_t CUSTOM_HID_OutEvent_FS(uint8_t* state);
	In the function, change: 	
		static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
		to static int8_t CUSTOM_HID_OutEvent_FS(uint8_t* state)

12. In the Main.c file:
	i. In the user-includes section add: #include "usbd_customhid.h"
	ii. In the user-private variables add:
		// This is for the USB
		extern USBD_HandleTypeDef hUsbDeviceFS;

		// Buffer for data transfer from USB to UART
		uint16_t tx_buffer_size=3;
		uint8_t tx_buffer[3];
		
		// Buffer to receive UART-Data
		uint16_t uart_rx_buffer_size=1;
		uint8_t uart_rx_buffer[1];

		// Buffer for data transfer from UART to USB
		uint16_t rx_buffer_size=4;
		uint8_t rx_buffer[4];

		// Variables to keep track of the current MIDI-command/status and the count of MIDI-msg. bytes
		uint8_t tx_note_status=0;
		uint8_t rx_note_status=0;
		int rx_data_count=0;
	iii. In the User-Code section-2 add:
		// Disable the half-transfer Interrupt for Uart-Tx, this is generally useful for circular buffer, but we are not using that here
		__HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);

		//Enable the Interrupt, such that when the RX-buffer is filled or its idle for one-uart-frame-time, a callback-function is called
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_buffer, uart_rx_buffer_size);
		// Disable the half-transfer Interrupt for Uart-Rx, this is generally useful for circular buffer, but we are not using that here
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	iv. In the while(1) loop, we will just put a HAL_Delay(10); for now, because most of the work will be done in the call-back functions.
	v. In the User-Code section-4, we will add the call-back functions to the UART receive and UART transmit, as follows:

		// Callback function for the UART-DMA-Receive
		void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
		{
			// Here we are only considering the 3-byte midi messages such as Note-On/Note-Off, pitch-bend, control change, poly-pressure etc
			// And if the consecutive MIDI-msgs are of same type, say Note-ON, then after the first Not-On status byte, only the data-bytes are send,
			// The following if-else and the count takes care of such messages, by saving the status byte, and updating only if new status-byte is received
			// Note, all MIDI data-bytes are 7-bit, i.e., of range 0x00 to 0x7F, and all midi-status are >= 0x80.
			// USB MIDI Packet: <(CABLE-NUM-NIBBLE)(MIDI-STATUS-NIBBLE)> <(MIDI-STATUS-NIBBLE)(MIDI-CHANNEL-NIBBLE)> <DATA-BYTE-1> <DATA-BYTE-2>
			// Ex.: 0x90 0x90 0x40 0x7F --> Note-ON for Note-64, at velocity 127 on channel-0 and cable-0.
			if ((uart_rx_buffer[0] >= 0x80) && (rx_data_count == 0))
			{
				rx_note_status = uart_rx_buffer[0];
				rx_data_count += 1;
				rx_buffer[0]=((rx_note_status >> 4) & 0x0F);
				rx_buffer[1]=rx_note_status;
			}
			else if ((uart_rx_buffer[0] < 0x80) && (rx_data_count == 0))
			{	
				rx_data_count += 2;
				rx_buffer[0]=((rx_note_status >> 4) & 0x0F);
				rx_buffer[1]=rx_note_status;
				rx_buffer[2]=uart_rx_buffer[0];
			}
			else if ((uart_rx_buffer[0] < 0x80) && (rx_data_count == 1))
			{
				rx_data_count += 1;
				rx_buffer[2]=uart_rx_buffer[0];
			}
			else if ((uart_rx_buffer[0] < 0x80) && (rx_data_count == 2))
			{
				rx_data_count = 0;
				rx_buffer[3]=uart_rx_buffer[0];
				// Send the MIDI-Message to Host-via USB
				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, rx_buffer, rx_buffer_size);
			}

			// Enable the Interrupt again
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_buffer, uart_rx_buffer_size);
			// Disable the half-transfer Interrupt, again
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		}

		// Callback function for the UART-DMA-Transmit
		void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
		{
			// Disable the half-transfer Interrupt for Uart-Tx
			__HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);
		}
12. In the usbd_custom_hid_if.c file:
	i. In the private-variables section add the following:
		extern UART_HandleTypeDef huart2;
		extern uint8_t tx_buffer[3];
		extern uint8_t tx_buffer_size;
		extern uint8_t tx_note_status;
	ii. In the CUSTOM_HID_OutEvent_FS function (User-Code section-6) add the following lines of code:
		// Send the status-byte of the total-midi-msg. only if its different from the last
		if (tx_note_status != state[1])
		{
			tx_note_status=state[1];

			/* To take care of 3-byte messages (status + 2byte-data):
			 	0x80 --> Note-Off
			 	0x90 --> Note-On
			 	0xA0 --> Poly-pressure
			 	0xB0 --> Control-change
			 	0xE0 --> Pitch-bend
			 	0xF2 --> Song-position
			*/
			if (!(tx_note_status ^ 0x80) || !(tx_note_status ^ 0x90) || !(tx_note_status^ 0xA0) || !(tx_note_status ^ 0xB0) || !(tx_note_status ^ 0xE0) || !(tx_note_status ^ 0xF2))
			{
				tx_buffer[0] = state[1];
				tx_buffer[1] = state[2];
				tx_buffer[2] = state[3];
				HAL_UART_Transmit_DMA(&huart2, tx_buffer, 3);
			}
		}
		else
		{
			/* To take care of 3-byte messages (status + 2byte-data):
			 	0x80 --> Note-Off
			 	0x90 --> Note-On
			 	0xA0 --> Poly-pressure
			 	0xB0 --> Control-change
			 	0xE0 --> Pitch-bend
			 	0xF2 --> Song-position
			*/
			if (!(tx_note_status ^ 0x80) || !(tx_note_status ^ 0x90) || !(tx_note_status ^ 0xA0) || !(tx_note_status ^ 0xB0) || !(tx_note_status ^ 0xE0) || !(tx_note_status ^ 0xF2))
			{
				tx_buffer[0] = state[2];
				tx_buffer[1] = state[3];
				HAL_UART_Transmit_DMA(&huart2, tx_buffer, 2);
			}
		}
