## __OUT16A1/DaisyChain Example Description__

Example project providing example code for driving IPS8200HQ, Octal High-Side Intelligent Power Switch with serial/parallel selectable interface on-chip
(to be used via expansion board X-NUCLEO-OUT16A1)

This firmware package supports the NUCLEO-G431RB Board and includes Components Device Drivers, Board Support Package
and example application for the following STMicroelectronics devices:

  - X-NUCLEO-OUT16A1 Expansion board using IPS8200HQ high-side IPS (Octal channel Intelligent Power Switch configured with serial interface and Daisy Chain)

\

### __Keywords__

IPS, high-side, SPI, IPS8200HQ, Intelligent Power Switch, Daisy Chain

\

### __Directory contents__

Project files are reported below:

| File                                                                                                                         | Description                                                  |
| :--------------------------------------------------------------------------------------------------------------------------- | :----------------------------------------------------------- |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/main.h                                                                | Header for main.c module                                     |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/app_ips_relay.h                                                       | Header for app_ips_relay.c module                            |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/app_ips_custom.h                                                      | Header for app_ips_custom.c module                           |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/app_ips_conf.h                                                        | Header for application configuration                         |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/app_ips_version.h                                                     | Header for application versioning                            |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/ips8200hq_conf.h                                                      | Header for BSP/Components/ips8200hq driver configuration     |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/out16a1_conf.h                                                        | Header for BSP/OUT16A1 driver configuration                  |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/stm32g4xx_nucleo.h                                                    | Header for stm32g4xx_nucleo.c module                         |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/stm32g4xx_nucleo_bus.h                                                | Header for stm32g4xx_nucleo_bus.c module                     |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/stm32g4xx_nucleo_conf.h                                               | Header for STM32G4xx nucleo configuration                    |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/stm32g4xx_hal_conf.h                                                  | HAL configuration file for STM32G4xx                         |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/stm32g4xx_it.h                                                        | Interrupt handlers header file for STM32G4xx                 |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/stm32g4xx_nucleo_errno.h                                              | Error codes for STM32G4xx-Nucleo                             |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/STMicroelectronics.X-CUBE-IPS_conf.h                                  | SW Pack configuration header file                            |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Inc/RTE_Components.h                                                      | Header for STM32CubeMX environment configuration             |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Src/main.c                                                                | Main program                                                 |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Src/app_ips_relay.c                                                       | Code for application example basics                          |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Src/app_ips_custom.c                                                      | Code for application example customization                   |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Src/stm32g4xx_hal_msp.c                                                   | HAL MSP module for STM32G4xx                                 |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Src/stm32g4xx_it.c                                                        | Interrupt handlers for STM32G4xx                             |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Src/stm32g4xx_nucleo.c                                                    | Code for STM32G4xx nucleo board management                   |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Src/stm32g4xx_nucleo_bus.c                                                | Code for STM32G4xx nucleo board I/O management               |
| Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/Src/system_stm32g4xx.c                                                    | System source file for STM32G4xx                             |

\

### __Hardware and Software environment__

This application example requires:

  - NUCLEO-G431RB board: a Nucleo development board for STM32
  - two X-NUCLEO-OUT16A1 boards: an expansion board based on IPS8200HQ (Octal channel high-side driver with serial/parallel selectable interface on-chip, smart driving 0.7A loads)

ADDITIONAL_BOARD: X-NUCLEO-OUT16A1 https://www.st.com/en/ecosystems/x-nucleo-out16a1.html

ADDITIONAL_COMP : IPS8200HQ https://www.st.com/en/power-management/ips8200hq.html

\

The system demonstrates a 16-channel digital output module.<br>
For more details, please read carefully the dedicated section for Daisy Chain configuration in the following HTML Help file:

- Documentation/X-CUBE-IPS_NUCLEO-G431RB.chm

\

This application example has been tested on the following IDE:

- IAR Embedded Workbench for ARM (EWARM) toolchain V9.20.1 + ST-LINK/V3
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.37.0.0 + ST-LINK/V3
- STM32CubeIDE for STM32 V1.15.1 + ST-LINK/V3
- STM32CubeMX (STM32Cube initialization code generator) V6.11.0

  > NOTE:
  > The firmware can be flashed on the NUCLEO-G431RB Board both using an IDE (IAR, Keil, STM32CubeIDE)
  > starting by the source code or using the binary file and STM32CubeProgrammer utility.
  > In the first case it`s important that the IDE debug environmnent is stopped.
  > In both cases, after flashing the firmware into the nucleo board, press the black button in it
  > to ensure that the firmware is correctly restarted.

\

#### __X-NUCLEO-OUT16A1 DaisyChain jumpers Setup__

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

__(Daisy Chain conf specific settings)__

  - SW4 Closed 2-3
  - SW5 Closed 2-3
  - SW6 (**DAISY_CHAIN/MOSI**)
    - Board 0:
      + Closed 2-3
    - Board 1:
      + Closed 1-2
  - SW7 Closed 2-3
  - SW9 Closed 2-3
  - SW10 Closed 2-3
  - SW11 Closed 2-3
  - SW12 Closed 2-3
  - SW13 Closed 2-3
  - SW14 Closed 2-3
  - SW15 Closed 2-3
  - SW18 (**SPI_MISO/DAISY_CHAIN**)
    - Board 0:
      + Closed 2-3
    - Board 1:
      + Closed 1-2
  - SW20 Closed 2-3

  - JP21 Closed
  - JP22
      + Open    SEL1 L (SPI 8 bits)
      + Closed  SEL1 H (SPI 16 bits)

  > NOTE:<br>
  > The expansion boards use only Arduino connectors, the morphos are not mounted.

\

#### __NUCLEO-G431RB jumpers Setup__

Setup for firmware flashing

  - JP5: closed 1-2 (5V_STLK for firmware flashing)
  - JP1, JP7 open
  - JP3, JP6 closed
  - JP8 closed 1-2
  - CN4 open
  - CN11 closed
  - CN12 closed

\

#### __Example Project__

The Example application initializes all the firmware components allowing the user to try
the IPS main functionalities like Steady State and PWM mode.<br>
The firmware supports two expansion boards X-NUCLEO-OUT16A1 properly configured in Daisy Chain Mode.

A command sequence is embedded into the firmware and the user can advance the sequence
by pressing the blue user button on the nucleo board.

Briefly, the order of functions implemented for all expansion boards is:

```
  1.  Enable outputs in all boards (common OUT_EN high)
      Set ON IN1, IN4, IN5, IN8 in board 0
      Set ON IN2, IN3, IN6, IN7 in board 1
  2.  Set ON IN2, IN3, IN6, IN7 in board 0
      Set ON IN1, IN4, IN5, IN8 in board 1
  3.  Set OFF IN1, IN2, IN5, IN6 in board 0
      Set OFF IN3, IN4, IN7, IN8 in board 1
  4.  Set OFF IN3, IN4, IN7, IN8 in board 0
      Set OFF IN1, IN2, IN5, IN6 in board 1
  5.  Set OFF IN1, IN2, IN3, IN4, and ON IN5, IN6, IN7, IN8 in board 0
      Set ON IN1, IN2, IN3, IN4 and OFF IN5, IN6, IN7, IN8 in board 1
  6.  Set ON IN1, IN2, IN3, IN4 and OFF IN5, IN6, IN7, IN8 in board 0
      Set OFF IN1, IN2, IN3, IN4 and ON IN5, IN6, IN7, IN8 in board 1
  7.  Set OFF all inputs in board 0 and board 1
      Start PWM on all inputs in board 0 and board 1 with different frequency and duty cycle settings.
      Details:
      - board 0 IN1, IN3, IN5, IN7: PWM ON with freq 2Hz DC 25%
      - board 0 IN2, IN4, IN6, IN8: PWM ON with freq 1Hz DC 50%
      - board 1 IN1, IN3, IN5, IN7: PWM ON with freq 1Hz DC 50%
      - board 1 IN2, IN4, IN6, IN8: PWM ON with freq 2Hz DC 25%
  8.  board 0 IN1, IN3, IN5, IN7: set DC 50%
      board 1 IN2, IN4, IN6, IN8: set DC 50%
  9.  board 0 IN2, IN4, IN6, IN8: set DC 75%
      board 1 IN1, IN3, IN5, IN7: set DC 75%
  10. board 0 IN1, IN3, IN5, IN7: set DC 100%
      board 1 IN2, IN4, IN6, IN8: set DC 100%
  11. board 0 IN2, IN4, IN6, IN8: set DC 100%
      board 1 IN1, IN3, IN5, IN7: set DC 100%
  12. Disable outputs in all boards (common OUT_EN low)
      Stop PWM on all inputs in board 0 and board 1
```

Each user blue button pressure moves the firmware to the next function.
The sequence is cyclic: after the last step (number 12) it returns to the first one (number 1).

\

### __How to use it?__

This package contains projects for 3 IDEs viz. IAR, µVision and STM32CubeIDE
In order to make the program work, you must do the following:

For IAR:

  - Open IAR toolchain
  - Open the IAR project workspace file Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/EWARM/Project.eww
  - Rebuild all files and flash the firmware with Download and Debug. Then Stop Debugging, reset the
    nucleo board with black button
  - Use the blue user button to try all the functionalities

For µVision:

  - Open µVision toolchain
  - Open the µVision project file Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/MDK-ARM/DaisyChain.uvprojx
  - Rebuild all files and use Download to flash the nucleo board
  - Reset the nucleo board with black button and try all the functionalities with the blue user button

For STM32CubeIDE:

  - Open STM32CubeIDE toolchain
  - Set your default workspace as requested by the IDE (please be sure that there are not spaces in the workspace path)
  - Double-click on project file Projects/NUCLEO-G431RB/Examples/OUT16A1/DaisyChain/STM32CubeIDE/.project
  - Rebuild all files and flash the nucleo board, then use Run to download the firmware.
  - Reset the nucleo board with black button and try all the functionalities with the blue user button

> IMPORTANT NOTE:<br>
> before opening the project with any toolchain be sure your folder installation path is not too in-depth since the toolchain may report errors after building.
