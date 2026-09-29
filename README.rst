The Skylo field-test sample demonstrates how to communicate with TagoIO using TIP protocol from the LooUQ MTC2-N9151 embedded modem.

Requirements
************

The sample supports the following development kits: LooUQ MTC2-N9151 embedded modem

Support for the modem (power, GPIO, etc.) via the LooUQ Breakout or UXplor development boards is required to run the sample. Alternatively, you can use your own custom board with the modem, but you must ensure that the modem is powered and connected to the host MCU via UART. 

The necessary `board files <https://github.com/LooUQ/LooUQ-MTC2-SW/tree/main/boards>`_ are available on GitHub for the MTC2-N9151 embedded modem. 

The configuration for the sample is provided in the sample's `prj.conf` file. If you are using a custom board, you may need to modify the configuration to match your board's hardware.

Overview
********

The Skylo sample creates a UDP configured PDN using the specified APN and uses socket operations to send and receive data. The diaglog is between the device and the TagoIO server, using the TIP protocol.

.. note::

   This sample requires a SIM subscription with a carrier such as Monogoto (or another carrier of your choice), and LTE network configured to route the non-IP traffic to a server that is able to respond.

Configuration
*************
The sample can be configured using numerous config options. The configuration options are defined in the sample's `prj.conf` file. You can modify the configuration options to match your requirements.

TagoIO credentials are kept out of the repository. Copy `secrets.conf.example` to `secrets.conf` and set `CONFIG_TAGO_DEVICE_TOKEN` and `CONFIG_TAGO_HASH`. `secrets.conf` is ignored by git and merged into the build automatically.


Building and running
********************
The sample can be built and run using the Nordic Connect SDK (NCS) build system. The following instructions assume that you have already set up the NCS environment and have the necessary tools installed. 

Testing
=======
To test the sample, follow these steps:

1. Connect the development kit to your host computer and ensure that it is powered on. 
2. Setup your terminal emulator to connect to the development kit's serial port. The default baud rate is 115200, and the default settings are 8 data bits, no parity, and 1 stop bit.
3. Build and flash the sample to the development kit using the NCS build system.
4. Sit back and observe the output in your terminal emulator. The sample will automatically connect to the network, create a socket, send a message, receive a response, and then close the socket. 
5. Open your TagoIO account and navigate to the "Devices" section. You should see the device listed there, and you can view the data that was sent and received by the device. 


Dependencies
************

