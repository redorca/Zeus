1. sercon
   This example will register the CDC ACM class as the device node "/dev/ttyACM0" in
   baud rate 115200bps, then you can use "/dev/ttyACM0" as uart for communicate with
   Host
   Step: 1) connect the usb cable between host and device usb port;
         2) nsh> sercon
         3) make sure the UART port had been recognized by Host
         4) you can use this new UART port
         5) simple test "nsh> echo "1234" > /dev/ttyACM0 "

2. serdis
   This example will un-register the device node of CDC ACM class
   STEP: 1) you should disconnect the USB Cable firstly
         2) use "nsh>serdis" to un-register the device node of CDC ACM class

3. usbserial
   This example as usb serial test, it will register CDC ACM class and it will send
   "Hello World" to Host, receive from Host.(baud rate is 115200)
