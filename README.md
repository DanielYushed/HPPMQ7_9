# Homotopic Path Planning for Robotics (HPPM)
### Optimized Hardware and Software Co-design

Hello! This repository contains a complete system for autonomous robot navigation. The best part about this project is how it combines the flexibility of software testing with the raw speed of hardware execution. 

We use the Homotopic Path Planning Method to find safe routes from a starting point to a goal. The algorithm uses a Newton-Raphson numerical method to avoid solid obstacles dynamically. We built the core hardware using fixed-point arithmetic (Q7.9 format). This makes the system incredibly fast and lightweight.

## Project Structure

I organized the code into three main sections so you can find exactly what you need.

### 1. Simulation Tools (The HPPM folder)
This folder holds the software models and your testing environment. Use these tools to test your ideas on a computer before moving to the physical board.
* `HPPM.c`: This is the baseline mathematical model. It uses standard floating-point numbers. It is great for understanding the pure algorithm.
* `HPPM_FPGA.c`: This is the fixed-point version. It acts as an exact software mirror of the hardware circuit. It includes all the mathematical clamps and optimizations we use to save resources on the board.
* `CorrerTodo.py`: A Python automation script. It runs your compiled programs against different maps and configuration files. After running the tests, it draws the robot trajectories and saves them as images for quick review.

### 2. Hardware Architecture (Verilog RTL)
These files describe the digital circuits. They are highly optimized for boards with limited logic elements and multipliers, like the Altera Cyclone II.
* **Master Control**: `hppm_top.v` and `hppm_core.v`. These files run the main state machine and coordinate all the sub-modules.
* **Repulsive Calculation**: `repulsive_acc.v`. This module calculates how obstacles push the robot away. It includes a mathematical wall to prevent dividing by zero during collisions.
* **Linear Algebra**: `newton_step.v` and `jacob_assembler.v`. They build and solve the 3x3 matrix using Cramer's Rule. They feature a dynamic row scaler to prevent bit overflows.
* **Math Functions**: `fx_norm3d.v` and `fx_luts.v`. They calculate 3D geometry and angles natively without using floating-point units.
* **Memory and Serial Port**: `uart_rx.v`, `uart_tx.v`, and `dual_port_ram.v`. They manage the communication with your computer and store the coordinates of up to 200 obstacles.

### 3. Control Interface
* We included a MATLAB script to manage the hardware. You can use it to draw obstacle maps, send them to the FPGA via UART, and plot the real-time path the chip calculates.

## How to use this project

### Running the PC Simulations
1. Create the input folders `inputs_pendientes` and `inputs_obstaculos`. Place your text configuration files inside them.
2. Compile the C files into executables and place them in the `output` folder.
3. Run the `CorrerTodo.py` script. 
4. Open the `resultados_img` folder to see the drawn maps and verify your robot paths.

### Running the FPGA Hardware
1. Open Quartus II and create a new project for your specific board.
2. Add all the `.v` files to your project.
3. Assign the physical pins for the clock, the reset button, and the UART communication lines.
4. Compile the project and upload the file to your board.
5. Open the MATLAB script, set your COM port, and run it. You will see the FPGA receive the map and return the calculated path instantly.

## Hardware Optimization Notes

We designed the hardware blocks strictly to save space and power. 
* **Matrix Scaling**: The linear algebra module checks the numbers before doing matrix multiplication. If the numbers grow too large, it shifts them down. This keeps all matrix operations strictly within 16 bits and saves dozens of hardware multipliers.
* **Force Pre-shifting**: The repulsive calculation scales down raw forces before multiplying them with physical distances. This avoids 64-bit numbers entirely and keeps the data bus clean and fast.
