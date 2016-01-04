## Background on Issues ##

### USB Suspend ###
(from FAQ)
```
If the USB data bus is idle for >3ms the USB host should put the FTxxx into 
suspend.

To prevent this happening you should ensure there is always data within 3ms. 
This could be done by sending dummy data.

The alternative sneaky fix is to set the latency timer to 2ms. This forces 
the 2 status bytes to be returned every 2ms and hence creates enough traffic 
to keep the device awake.
```

### Latency ###
(from AN232B-04)
```
If data is to be sent from the PC, then a packet of data is built up by the 
device driver and sent to the USB scheduler. This scheduler puts the request 
onto the list of tasks for the USB host controller to perform. This will 
typically take at least 1 millisecond to execute because it will not pick up 
the new request until the next 'USB Frame' (the frame period is 1 millisecond)
```

```
Data is received from USB to the PC by a polling method. The driver will 
request a certain amount of data from the USB scheduler. This is done in 
multiples of 64 bytes (64-4k). The 'bulk packet size' on USB is a maximum 
of 64 bytes. The host controller will read data from the device until 
either:

a) a packet shorter than 64 bytes is received or
b) the requested data length is reached
```

```
When transferring data from an FTDI USB-Serial or USB-FIFO IC device to the 
PC, the device will send the data given one of the following conditions:

1. The buffer is full (64 bytes made up of 2 status bytes and 62 user bytes).
2. One of the RS232 status lines has changed (USB-Serial chips only).
    (CTS#/DSR#/DCD#/RI#)
3. An event character had been enabled and was detected in the incoming data 
stream.
4. A timer integral to the chip has timed out. There is a timer (latency timer) 
in the FT232R, FT245R, FT2232C, FT232BM and FT245BM chips that measures the time
since data was last sent to the PC. The default value of the timer is set to 16
milliseconds. Every time data is sent back to the PC the timer is reset. If it
times-out then the chip will send back the 2 status bytes and any data that is 
held in the buffer.
```

```
A worst case condition could occur when 62 bytes of data are received in 16
milliseconds. This would not cause a timeout, but would send the 64 bytes (2 
status + 62 user data bytes) back to USB every 16 milliseconds. When the USBD
system driver receives the 64 bytes it would hold on to them and request another 
'IN' transaction. This would be completed another 16 milliseconds later and so on 
until USBD gets all of the 4K of data required. The overall time would be (4096 /
64) * 16 milliseconds = 1.024 seconds between data packets being received by the 
application. In order to stop the data arriving in 4K packets, it should be 
requested in smaller amounts. A short packet (< 64 bytes) will of course cause 
the data to pass from USBD back to the FTDI driver for use by the application.
```
