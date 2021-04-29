# COMPSYS 723 Assignment 1 - Group 10

# Setup Instructions

### Board Setup

1. Connect the 12V adapter to the DE2-115 board.
2. Connect the VGA from the DE2-115 board to a VGA monitor.
3. Connect the PS2 keyboard to the DE2-115 board.
4. Connect the USB-Blaster to the DE2-115 board.
5. Power on the board.

### Project Setup
1. Using the <b> Programmer (Quartus Prime 18.1)</b> application, program the DE2-115 board using the <b><em>freq_relay_controller.sof</em></b> found in the <b><em>COMPSYS723 2021 FreeRTOS Resources.zip</em></b> provided on Canvas.
2. Open <b>Nios II Software Build Tools for Eclipse(Quartus Prime 13.0)</b> and create a workspace on D: drive.
3. Select <b><em>File -> New -> Nios II Application and BSP From Template</em></b>
4. Specify the .sopcinfo file using the provided <b><em>nios2.sopcinfo</em></b> file found on Canvas.
5. Give the project an appropriate name.
6. Select <b><em>Blank Project</em></b> as the Project Template.
7. Click <b><em>Finish</em></b>

### Program Setup
1. Copy the <b><em>FreeRTOS</em></b> folder provided on Canvas to the <em>folder of the application software</em>. This folder is named the same as your project name.
2. Copy the <b><em>freq_relay_controller.c</em></b> to the folder of the application software.
3. Press <b><em>CTRL + B</em></b> to build the project. 
4. Select <b><em>Run -> Run Configurations...</em></b>
5. Under the <b><em>Project</em></b> tab, select the created project and the associated project ELF file.
6. Under the <b><em>Target Connection</em></b> tab, ensure the USB Blaster displays under both the <b><em>Processor and Byte Stream Devices</em></b>
7. Select <b><em>Run</em></b>.

# I/O 

| IO Name         | Functionality |
| -------------       |:-------------:| 
| Keyboard         | Pressing the +/- keys closest to the NUMPAD will increase/decrease the ___ threshold respectively. Pressing the +/- keys closest to the BACKSPACE key will increase/decrease the __ threshold respectively| 
| SW[4] - SW[0]      | Slding the switch up will turn on the load whereas sliding the switch down will turn off the load.      |  
| Green LEDS | A lit Green LED will indicate that the relay controller has disconnected the load automatically.    |   
| Red LEDS | A lit Red LED will indicate that either the relay controller has either reconnected the load automatically or the user has manually turned the load on using the associated switch     |   

# Functionality
## Initial Thresholds
+ Rate of Change (RoC) of Frequency Threshold - 10 
+ Frequency Threshold - 49 Hz

## Modes
### <b>Maintenance</b>
  
In Maintenance Mode, the user is able to configure the thresholds of the system by using the respective keys mentioned in [I/O](#io) 
### <b>Load Manage</b>

In Load Manage Mode, the system will either shed or reconnect loads depending on the stability of the system. If the system is unstable due to the thresholds being violated, the system is deemed unstable and will shed loads. The first load will be shed instantly whereas the remaining loads will be shed if the system has detected the system has remained unstable for 500ms. The system is deemed stable when both the measured Frequency and RoC of Frequency do not violate the thresholds for 500ms. Under stable conditions, the system will reconnect any shed loads until all the loads are connected.

In this mode, the user is able to manually drop loads by sliding the associated switch down. The user is unable to reconnect loads in this mode. 

### <b>Stable</b>
Once all the loads are reconnected, and the system is stable for an additional 500ms, the system is in Stable Mode. In Stable Mode, the user is able to manually connect and drop loads to the system by sliding the switch up or down respectively. 


