README
======

This README discusses issues unique to NuttX configurations for the 
nRF52_DK Board (PCA10040) featuring the nRF52832 MCU. the nRF52832 is a 64MHz
Cortem-M4 with FPU and 512KB flash memory and 64Kbytes RAM. 
The board features:

  - nRF52832 flash-based ANT/ANT+, Bluetooth® low energy SoC solution
  - Buttons and LEDs for user interaction
  - I/O interface for Arduino form factor plug-in modules
  - SEGGER J-Link OB debugger with debug out functionality
  - Virtual COM port interface via UART
  - Drag-and-drop mass storage device (MSD) programming
  - Supporting NFC-A listen mode

Refer to http://www.nordicsemi.com/start52dk for more information about this board.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - UARTS
  - Fat FileSystem
  - PWM
  - ADC
  - SPI

Development Environment
======

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
======
  Using the pre-built GNU gcc toolchain(gcc-arm-none-eabi). You can download it from 
  https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads

UARTS
======
  Please don't use below GPIO, they are connectted to Interface MCU by hardware.

	P0.05 RTS
	P0.06 TXD
	P0.07 CTS 
	P0.08 RXD

FAT FileSystem
======
  It supportted to use onchip flash to create FAT file system. You can open it by below
CONFIG_ANALOG=y
CONFIG_ADC=y
CONFIG_ADC_FIFOSIZE=8
  macro.

	+CONFIG_MTD=y
	+CONFIG_MTD_PARTITION=y
	+CONFIG_MTD_PARTITION_NAMES=y
	+CONFIG_MTD_BYTE_WRITE=y
	+CONFIG_MTD_PROGMEM=y
	+CONFIG_FS_FAT=y
	+CONFIG_FAT_LCNAMES=y
	+CONFIG_FAT_LFN=y
	+CONFIG_FAT_MAXFNAME=32
	+CONFIG_FS_PROCFS=y
	+CONFIG_FS_PROCFS_REGISTER=y
	+CONFIG_EXAMPLES_MOUNT=y
	+CONFIG_EXAMPLES_MOUNT_BLOCKDEVICE=y
	+CONFIG_EXAMPLES_MOUNT_DEVNAME="/dev/mtdblock1"

  nsh>flash_eraseall /dev/mtdblock1
  nsh>mkdir /fatfs
  nsh>mkfatfs /dev/mtdblock1
  nsh>mount -t vfat /dev/mtdblock1 /fatfs
  nsh>mount  # check if the fat file system is mounted
  nsh>cd /fatfs
  nsh>echo 123456 >test.txt

PWM
======


ADC
======
  Using MEXICO BOURNS for ADC test, you can adjust BOURNS to test ADC.
  Please open below option fro ADC and connect P0.03 to BOURNS.
 
	+CONFIG_NRF52_ADC=y
	+CONFIG_NRF52_ADC_CHANNEL=1

	+CONFIG_ANALOG=y
	+CONFIG_ADC=y
	+CONFIG_ADC_FIFOSIZE=8

	+CONFIG_EXAMPLES_ADC=y
	+CONFIG_EXAMPLES_ADC_DEVPATH="/dev/adc0"
	+CONFIG_EXAMPLES_ADC_GROUPSIZE=4
	+CONFIG_EXAMPLES_ADC_SWTRIG=y

  nsh>adc -n 1
  adc_main: g_adcstate.count: 1
  adc_main: Hardware initialized. Opening the ADC device: /dev/adc0
  adc_read: buflen: 20
  adc_read: Returning: 10
  Sample:
  1: channel: 0 value: 10608

SPI
======
  There are two spi driver , one is for easy DMA , another is legacy driver.
  The easy DMA has one issue for send one byte . The legacy driver is totally
  fine.
  For CS signal, if you connect SPI bus by one component, you can use kernel CS
  select function . If you connect same SPI bus by multi-component , you should
  define the CS and implemnet it in nrf52_user_spi.c.

  below is example to connect winbond w25q128 spi flash to SPI bus 2
	+CONFIG_NRF52_LEGACY_SPI2=y
	+CONFIG_NRF52_LEGACY_SPI=y
	+CONFIG_NRF52_SPI_INTERRUPTS=y
	+CONFIG_NRF52_CS_CONTROL_BY_USER=y
	+ONFIG_NRF52_SPI2_SCL_PIN=22
	+CONFIG_NRF52_SPI2_MOSI_PIN=23
	+CONFIG_NRF52_SPI2_MISO_PIN=25
	+CONFIG_NRF52_SPI2_8M=y

  There will be output from UART as below.

W25 Flash Geometry:
	blocksize:		256
	erasesize:		4096
	neeraseblocks:  4096

  Also , you can enable the mtdblock rw test example by below macro.

	+CONFIG_EXAMPLES_MTDBLOCKRW=y

nsh> mtdblock_rw -d /dev/mtdblock2 -n 1024
Flash Geometry:
  blocksize:      256
  erasesize:      4096
  neraseblocks:   4096
  blkpererase:    16
  nblocks:        65536

Starting to erase All Flash....

Erasing All Flash Done....
Initializing media:
Wrote Block 0  Done.
Wrote Block 1  Done.
Wrote Block 2  Done.
Wrote Block 3  Done.
Starting to verify writing data
Starint to verify Block 0 ...
Starint to verify Block 1 ...
Starint to verify Block 2 ...
Starint to verify Block 3 ...
PASS: Everything looks good

 



