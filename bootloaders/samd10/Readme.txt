A low footprint SERCOM UART based bootloader can be found in this attachment. 
This bootloader is compiled with ARMGCC compiler using Atmel Studio 7 and 
occupies only 1008 bytes.

Following are the steps to execute the bootloader.

1. Unzip the attachment to a folder and open the 'UART_Bootloader.atsln' project in 
Atmel Studio 7.

2. The default settings are for SAM D11 Xplained Pro board.
 
SERCOM2 with 115200 baud is used and PA10 (PAD2) is used as TXD and PA11 (PAD3) is used as RXD. 
Push button connected to pin PA14 is used for boot mode entry.
Change the SERCOM module and boot mode entry pins in the main.c file if needed for custom boards.

3. Build the project and flash the target SAM D11 device with the bootloader hex file.

4. Enter into bootloader mode by performing the following sequence in SAM D11 Xplained Pro board.

    a. Press and hold the RESET button as well as the user push button SW0.
    b. First release the RESET button and then release the SW0 push button.

5. A python script file (uart_bootloader.py) which acts as the bootloader frontend is 
available in the attachment.

6. Open a command prompt and navigate to the script file path. Place your target 
application .bin file in the same path.

7. Execute the following command line which will program the input binary file.
    python.exe uart_bootloader.py -c COM1 -b 115200 -i input_file.bin

Notes:

1. Python has to be installed to run the bootloader script file. 
Python version 2.7.1 and pyserial 2.7 win32 has been used for testing. 
Other versions may or may not work.

2. The project in the attachment has been tested on SAM D11 Xplained Pro boards and 
can be easily ported to custom hardware.

3. The flash programming algorithm is common for all SAM D MCUs and hence the bootloader 
can be easily ported and run in MCUs like SAM D20 / SAM D21 / SAM D10 and SAM D11 devices.