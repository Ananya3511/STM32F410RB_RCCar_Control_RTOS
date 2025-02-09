### Code Description

This code is designed to control a robot (or similar device) using a microcontroller, such as an STM32, via UART communication. The microcontroller listens for commands via UART to control the robot's movement and perform different actions. It uses FreeRTOS for task management and several threads to handle different commands concurrently.

#### Key Components:

1. **UART Communication**:
   - The microcontroller uses two UARTs (`USART1` and `USART2`). `USART1` is used for receiving commands from an external source (like a terminal or another device). The received data is stored in `RX_BUFFER`.
   - The `HAL_UART_Receive_IT` function is used to enable interrupt-based reception. When data is received on UART1, it triggers the `HAL_UART_RxCpltCallback()` function, where the received command is processed.
   
2. **RTOS Tasks**:
   - The system uses FreeRTOS to manage different tasks (`defaultTask`, `forward`, `right`, `left`, and `back`). Each task controls specific parts of the robot's motion.
   - The priority of each task is set based on the received UART commands. The system dynamically adjusts the task priority to perform actions like moving forward, left, right, or backward.

3. **GPIO Control**:
   - GPIO pins control the motion of the robot. When specific commands are received, the corresponding GPIO pins are set to HIGH or LOW to control the motors or actuators.
   - The control pins (like `GPIO_PIN_0`, `GPIO_PIN_1`, etc.) are toggled in each task based on the received command.

4. **Interrupt Handling**:
   - The `HAL_UART_RxCpltCallback()` function is used to process the received data asynchronously. This function is called when a new character is received over UART1.

5. **Commands**:
   - The system responds to the following commands:
     - **'F'**: Move forward.
     - **'B'**: Move backward.
     - **'L'**: Turn left.
     - **'R'**: Turn right.
     - **'W'**: Set a specific GPIO pin to high (`GPIO_PIN_0`).
     - **'w'**: Set a specific GPIO pin to low (`GPIO_PIN_0`).
     - Other commands result in the default task behavior.

### File Structure:

1. **`main.c`**:
   - Contains the main control loop, system initialization, FreeRTOS task creation, and UART interrupt handling.

2. **FreeRTOS Task Management**:
   - **`StartDefaultTask`**: This task runs by default and listens for commands that toggle GPIO pins.
   - **`forward`**, **`right`**, **`left`**, **`back`**: These tasks handle the specific robot movements based on the received commands.

3. **UART Communication**:
   - **`HAL_UART_RxCpltCallback`**: This function is invoked when UART1 receives data and sets the appropriate commands in `RX_BUFFER`.

### Readme

## Project: STM32 Robot Control

### Overview:
This project controls a robot or similar device using an STM32 microcontroller and FreeRTOS. It receives commands via UART communication and performs specific actions, including forward, backward, left, right, and GPIO control.

### Features:
- **UART Communication**: Receives commands from an external device.
- **FreeRTOS**: Manages multiple tasks concurrently to control robot movement.
- **GPIO Control**: Uses GPIO pins to control motors or actuators.
- **Interrupt Handling**: Asynchronous UART data reception.

### Command Set:
- **'F'**: Move forward.
- **'B'**: Move backward.
- **'L'**: Turn left.
- **'R'**: Turn right.
- **'W'**: Set GPIO pin high (`GPIO_PIN_0`).
- **'w'**: Set GPIO pin low (`GPIO_PIN_0`).

### Hardware Requirements:
- STM32 microcontroller with FreeRTOS enabled.
- Motors or actuators connected to specific GPIO pins.
- UART communication to receive commands (e.g., via USB-to-serial or Bluetooth).

### Software Setup:
1. **STM32CubeMX**: Use STM32CubeMX to configure the hardware settings, including UART, GPIO, and FreeRTOS.
2. **HAL Libraries**: Use STM32 HAL libraries for hardware abstraction.
3. **FreeRTOS**: Make sure FreeRTOS is enabled for task management.

### How to Use:
1. Connect the STM32 to a serial interface (e.g., USB-to-serial adapter or Bluetooth).
2. Open a serial terminal (like PuTTY or Tera Term).
3. Send one of the following commands:
   - **'F'**: Move forward.
   - **'B'**: Move backward.
   - **'L'**: Turn left.
   - **'R'**: Turn right.
   - **'W'**: Set GPIO pin high.
   - **'w'**: Set GPIO pin low.
4. The robot will perform the corresponding action based on the received command.

### System Initialization:
1. **System Clock**: Configured to run at high speed using PLL.
2. **GPIO**: Configured to control motors and other peripherals.
3. **UART**: Configured for receiving commands.

### Known Issues:
- Ensure that the microcontroller's UART baud rate matches the terminal's baud rate (e.g., 9600 for UART1).
- The system may not behave as expected if the hardware is not properly connected.

### License:
This software is provided under the terms in the LICENSE file.
### Conclusion:
This project enables you to control a robot using UART communication and FreeRTOS for task management. It demonstrates how to use interrupts, task scheduling, and GPIO control in an embedded system.
