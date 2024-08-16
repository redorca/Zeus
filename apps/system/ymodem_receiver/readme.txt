you can test the ymodem receiver app in the following steps:
1, cd configs/nrf52840_dk/
2, make ymodem_receiver
3, download the firmware image into the nrf52840 chip
4, connect the chip's usb port to your PC.
   If you are using the PCA10056 nordic dev board,connect the board's J3 to your PC.
5, start dev board and type the following commands in the console:
   "sercon"
   this will enable the usb CDC drivers and you will see a new serial port appear in your PC.
6, setup the terminal software:
   a, choose the new serial port of step 5 and set its parameter:115200bps,8N1
   b, start ymodem sender
    If you are using secureCRT, you can do it like this:
    open menu: Transfer -> Send Ymodem
    select the file you want to transfer, and press OK.
7, type the following comands in the dev board's console:
   a, "ymodem_receiver"
    this will start the ymodem receiver app.
   b, "start  PATHXXXX"
    this command will start the ymodem transfer, "PATHXXXX" stands for the path you want to place the received file
    If all these commands execute successfully, you will see the complete rate of transfer process.
