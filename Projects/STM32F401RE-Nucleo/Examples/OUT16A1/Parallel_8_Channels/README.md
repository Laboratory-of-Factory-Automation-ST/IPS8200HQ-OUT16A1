## __OUT16A1/Parallel_8_Channels Example Description__

Example project providing example code for driving IPS8200HQ, Octal High-Side Intelligent Power Switch with serial/parallel selectable interface on-chip
(to be used via expansion board X-NUCLEO-OUT16A1)

This firmware package supports the NUCLEO-F401RE Board and includes Components Device Drivers, Board Support Package
and example application for the following STMicroelectronics devices:

  - X-NUCLEO-OUT16A1 Expansion board using IPS8200HQ high-side IPS (Octal channel Intelligent Power Switch configured with parallel interface)

\

### __Keywords__

IPS, high-side, IPS8200HQ, Intelligent Power Switch

\

### __Directory contents__

Project files are reported below:

| File                                                                                                                         | Description                                                  |
| :--------------------------------------------------------------------------------------------------------------------------- | :----------------------------------------------------------- |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/main.h                                                       | Header for main.c module                                     |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/app_ips_switch.h                                             | Header for app_ips_switch.c module                           |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/app_ips_custom.h                                             | Header for app_ips_custom.c module                           |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/app_ips_conf.h                                               | Header for application configuration                         |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/app_ips_version.h                                            | Header for application versioning                            |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/ips8200hq_conf.h                                             | Header for BSP/Components/ips8200hq driver configuration     |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/out16a1_conf.h                                               | Header for BSP/OUT16A1 driver configuration                  |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/stm32f4xx_nucleo.h                                           | Header for stm32f4xx_nucleo.c module                         |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/stm32f4xx_nucleo_conf.h                                      | Header for STM32F4xx nucleo configuration                    |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/stm32f4xx_hal_conf.h                                         | HAL configuration file for STM32F4xx                         |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/stm32f4xx_it.h                                               | Interrupt handlers header file for STM32F4xx                 |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/stm32f4xx_nucleo_errno.h                                     | Error codes for STM32F4xx-Nucleo                             |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/STMicroelectronics.X-CUBE-IPS_conf.h                         | SW Pack configuration header file                            |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Inc/RTE_Components.h                                             | Header for STM32CubeMX environment configuration             |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Src/main.c                                                       | Main program                                                 |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Src/app_ips_switch.c                                             | Code for application example basics                          |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Src/app_ips_custom.c                                             | Code for application example customization                   |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Src/stm32f4xx_hal_msp.c                                          | HAL MSP module for STM32F4xx                                 |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Src/stm32f4xx_it.c                                               | Interrupt handlers for STM32F4xx                             |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Src/stm32f4xx_nucleo.c                                           | Code for STM32F4xx nucleo board management                   |
| Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/Src/system_stm32f4xx.c                                           | System source file for STM32F4xx                             |

\

### __Hardware and Software environment__

This application example requires:

  - NUCLEO-F401RE board: a Nucleo development board for STM32
  - X-NUCLEO-OUT16A1 board: an expansion board based on IPS8200HQ (Octal channel high-side driver with serial/parallel selectable interface on-chip, smart driving 0.7A loads)

ADDITIONAL_BOARD: X-NUCLEO-OUT16A1 https://www.st.com/en/ecosystems/x-nucleo-out16a1.html

ADDITIONAL_COMP : IPS8200HQ https://www.st.com/en/power-management/ips8200hq.html

\

For more details, please read carefully the dedicated section in the following HTML Help file:

- Documentation/X-CUBE-IPS_NUCLEO-F401RE.chm

\

This application example has been tested on the following IDE:

- IAR Embedded Workbench for ARM (EWARM) toolchain V9.20.1 + ST-LINK/V2
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.37.0.0 + ST-LINK/V2
- STM32CubeIDE for STM32 V1.15.1 + ST-LINK/V2
- STM32CubeMX (STM32Cube initialization code generator) V6.11.0

  > NOTE:
  > The firmware can be flashed on the NUCLEO-F401RE Board both using an IDE (IAR, Keil, STM32CubeIDE)
  > starting by the source code or using the binary file and STM32CubeProgrammer utility.
  > In the first case it`s important that the IDE debug environmnent is stopped.
  > In both cases, after flashing the firmware into the nucleo board, press the black button in it
  > to ensure that the firmware is correctly restarted.

\

#### __X-NUCLEO-OUT16A1 Parallel_8_Channels jumpers Setup__

An example of board setup can be the following configuration:

__(Common settings)__

  - SW1 Closed 1-2
  - SW3 Closed 1-2
  - SW17 Closed 1-2

  - JP1, JP2, JP3, JP4, JP5, JP6, JP7, JP8 Closed to enable OUT1-8 output lines
  - JP9 Closed
  - JP10 Open
  - JP11 Not mounted
  - JP12 Closed
  - JP13 Closed
  - JP14 Open
  - JP15 Closed
  - JP16 Open
  - JP17 Open
  - JP18 Open
  - JP19 Open
  - JP20 Closed
  - JP23 Closed
  - JP24 Closed
  - JP25 Closed
  - JP27 Closed
  - JP28 Closed 2-4
  - JP29 Closed 1-2, 3-4, 5-6, 7-8 to enable active state led for OUT1-4
  - JP30 Closed 1-2, 3-4, 5-6, 7-8 to enable active state led for OUT5-8
  - JP31 Closed

__(Parallel 8 conf specific settings)__

  - SW4 Closed 1-2
  - SW5 Closed 1-2
  - SW6 Closed 1-2
  - SW7 Closed 1-2
  - SW9 Closed 1-2
  - SW10 Closed 1-2
  - SW11 Closed 1-2
  - SW12 Closed 1-2
  - SW13 Closed 1-2
  - SW14 Closed 1-2
  - SW15 Closed 1-2
  - SW18 Open
  - SW20 Closed 1-2

  - JP21 Open
  - JP22 Open

  > NOTE:<br>
  > The expansion boards use only Arduino connectors, the morphos are not mounted.

\

#### __NUCLEO-F401RE jumpers Setup__

Setup for firmware flashing

  - JP5 on U5V for firmware flashing
  - JP1 open
  - JP6 closed
  - CN2: closed 1-2, 3-4
  - CN3 open
  - CN4 open
  - CN11 closed
  - CN12 closed

\

#### __Example Project__

The Example application initializes all the firmware components allowing the user to try
the IPS main functionalities like Steady State and PWM mode.<br>
The firmware supports one expansion board, X-NUCLEO-OUT16A1.

A command sequence is embedded into the firmware and the user can advance the sequence
by pressing the blue user button on the nucleo board.

Briefly, the order of functions implemented for all expansion boards is:

```
  1.  Read board HW setup and set operating mode
      Set ON IN1, IN4, IN5, IN8
  2.  Set ON IN2, IN3, IN6, IN7
  3.  Set OFF IN1, IN2, IN5, IN6
  4.  Set OFF IN3, IN4, IN7, IN8
  5.  Set OFF IN1, IN2, IN3, IN4, Set ON IN5, IN6, IN7, IN8
  6.  Set ON IN1, IN2, IN3, IN4, Set OFF IN5, IN6, IN7, IN8
  7.  Start PWM on all inputs with different frequency and duty cycle settings.
      Details:
      - IN1, IN3, IN5, IN7: PWM ON with freq 2Hz, DC 25%
      - IN2, IN4, IN6, IN8: PWM ON with freq 1Hz, DC 50%
  8.  IN1, IN3, IN5, IN7: set DC 50%
  9.  IN2, IN4, IN6, IN8: set DC 75%
  10. IN1, IN3, IN5, IN7: set DC 100%
  11. IN2, IN4, IN6, IN8: set DC 100%
  12. Stop PWM on all inputs
```

Each user blue button pressure moves the firmware to the next function.
The sequence is cyclic: after the last step (number 12) it returns to the first one (number 1).

\

### __How to use it?__

This package contains projects for 3 IDEs viz. IAR, µVision and STM32CubeIDE
In order to make the program work, you must do the following:

For IAR:

  - Open IAR toolchain
  - Open the IAR project workspace file Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/EWARM/Project.eww
  - Rebuild all files and flash the firmware with Download and Debug. Then Stop Debugging, reset the
    nucleo board with black button
  - Use the blue user button to try all the functionalities

For µVision:

  - Open µVision toolchain
  - Open the µVision project file Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/MDK-ARM/Parallel_8_Channels.uvprojx
  - Rebuild all files and use Download to flash the nucleo board
  - Reset the nucleo board with black button and try all the functionalities with the blue user button

For STM32CubeIDE:

  - Open STM32CubeIDE toolchain
  - Set your default workspace as requested by the IDE (please be sure that there are not spaces in the workspace path)
  - Double-click on project file Projects/NUCLEO-F401RE/Examples/OUT16A1/Parallel_8_Channels/STM32CubeIDE/.project
  - Rebuild all files and flash the nucleo board, then use Run to download the firmware.
  - Reset the nucleo board with black button and try all the functionalities with the blue user button

> IMPORTANT NOTE:<br>
> before opening the project with any toolchain be sure your folder installation path is not too in-depth since the toolchain may report errors after building.
