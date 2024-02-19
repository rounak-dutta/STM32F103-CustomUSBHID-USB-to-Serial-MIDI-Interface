# STM32F103-CustomUSBHID-USB-to-Serial-MIDI-Interface
In this project a very basic USB to Serial MIDI Interface is built with STM32F103C8T6 board, with USB Custom-HID Middleware implementation in STM CubeIDE environment.


**Notes:** Before going through the project and/ or trying to implement the project, I would request the viewer to go through **the following drawbacks or limitations of the project,** as follows:

  i. I currently do not own any MIDI instruments which support serial-MIDI, so I tested by connecting the uC TX-pin to the RX-pin with a piece of wire (i.e., loopback test), and sent MIDI data from DAW to the uC and back into the DAW to a virtual instrument, and it is able to relay most of the MIDI data. But in faster paced sections of the MIDI-song, there are few lost-notes/ lost-MIDI-data, thus the USB 2 Serial conversion is not 100% lossless, atleast in the loop-back test.
  
  ii. I have spent a considerable amount of time trying to debug the lost-message issue, I have tried Interrupt UART, DMA UART (as in current implementation), but nothing could get me 100% transmission and reception. If I place a USB send-report immedietly after the USB data receive (i.e., bypassing the UART), then I don't get the issue, thus the data-loss has something to do with the UART interface. If anyone finds a solution to this issue please feel free to leave a comment in this project or in the YouTube video associated with this.
  
  iii. In this implementation, I have only considered the 3-byte MIDI messages, which generally covers the Note-On, Note-Off, Sustain Pedal, Pitch-Bend, etc., i.e., the most common types of MIDI-messages. The 2-bytes, 1-byte and SysEx MIDI messages hadling is not implemented. 


**References:**
1. https://www.usb.org/sites/default/files/midi10.pdf
2. https://www.youtube.com/watch?v=3JGRt3BFYrM
3. https://www.youtube.com/watch?v=740XGkC0DfQ
4. https://learn.sparkfun.com/tutorials/midi-tutorial/all#shortcomings
5. https://www.cs.cmu.edu/~music/cmsip/readings/davids-midi-spec.htm
6. https://github.com/Hypnotriod/midi-box-stm32/blob/master/MDK-ARM/startup_stm32f103xb.s


**The steps to implement Custom-HID USB-MIDI for STM32 in CubeIDE, is in the "Steps4UsbMIDIInterface.txt" file.**
