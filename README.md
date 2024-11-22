# SPAC-R: Suction-Driven Paper Assembly and Crafting Robot

SPAC-R is a Delta robot designed to perform pick-and-place operations using a suction mechanism. It automates the assembly of paper craft flowers by picking pre-cut petals from a designated area, applying glue to them, and assembling them into a complete flower. This repository contains all the necessary resources to recreate and operate SPAC-R, including 3D models, source code, and reference materials.

---

## Features

- **Delta Robot Mechanism**: SPAC-R employs a 4-DOF Delta robot configuration for high-speed, precise movements.
- **Suction Mechanism**: A suction cup mounted on the end effector is used for picking and placing petals.
- **Automated Crafting**: The robot applies glue and assembles petals into a flower autonomously.
- **Inverse Kinematics**: Ensures smooth and accurate motion control of the end effector.
- **Arduino Nano Control**: The system is powered by an Arduino Nano, with code written in C++ for precise control of servo motors and suction mechanisms.

---

## Repository Structure

### 1. **STL Files**
- All 3D-printable components required to build SPAC-R are provided in the `STL_Files` folder.
- Components include:
  - Frame structure
  - End effector
  - Suction mechanism attachments
  - Joint connectors

### 2. **Source Code**
- **`DeltaKinematics.cpp`**: The primary code for controlling SPAC-R's operations.
- **DeltaKinematics.h (`.h`)**: Modularized code for inverse kinematics, servo control, and suction operations.
- Designed to run on an Arduino Nano microcontroller.
  

### 3. **Prototype Reference Images**
- Find progress images of SPAC-R in the `Progress Images` folder:
  - Progress2.jpg
  - Prototype Image.jpeg
  - ![image](https://github.com/user-attachments/assets/2d756417-e89e-4cf8-83a7-ecff5ec4b29a)

  - Suction cup of the end effector
  - ![image](https://github.com/user-attachments/assets/81ca00ac-1429-40cf-bb5c-52f0652061bc)

  - progress3.jpg
  - final model image
  - ![image](https://github.com/user-attachments/assets/0038c177-b69b-48da-bd95-c52fe95eae31)


### 4. **Reference Videos**
- Demonstrations of SPAC-R in action are located in the `reference Videos` folder:
  - Initial prototype testing
  - End effector rotation
  - Suction assembly in operation
  - Servo movement control
  - Final prototype assembling a flower

---

## Hardware Requirements

### Electronics
- **Controller**: Arduino Nano
- **Servo Motors**:
  - Three for the Delta robot's shafts
  - One micro servo for end effector rotation
- **Suction Mechanism**: Air compressor with battery for powering suction cup
- **Camera**: RPi Camera Module for petal detection (optional)

  ### Circuit Diagram
  ![image](https://github.com/user-attachments/assets/462c7e7a-abc2-4351-8163-673405f6beee)


### 3D-Printed Parts
- STL files are included for all major structural components.

---

## Software Requirements

1. **Arduino IDE**: To upload the control code to the Arduino Nano.
2. **VarSpeedServo Library**: For smooth servo transitions during movement.
3. **C++ Programming Knowledge**: For understanding and customizing the source code if needed.

---

## Assembly Instructions

1. **3D Print Components**: Print the STL files provided in the `STL_Files` folder.
2. **Assemble the Delta Robot**:
   - Follow the reference images in the `Images` folder for guidance.
3. **Install Electronics**:
   - Connect the servo motors and suction mechanism to the Arduino Nano.
   - Attach the camera module (if used).
4. **Upload the Code**:
   - Use Arduino IDE to compile and upload the code from `main.cpp`.
5. **Test the System**:
   - Run initial tests using the `Videos` folder as a guide.

---

## How It Works

1. **Initialization**: SPAC-R moves to the home position and waits for a command.
2. **Pick Operation**: The suction cup picks a petal from the pre-cut paper area.
3. **Glue Application**: The petal is moved to the gluing area.
4. **Assembly**: The petals are sequentially placed to form a complete flower.

---

## List Of Components
![image](https://github.com/user-attachments/assets/5058327a-3945-499f-8821-78734848723b)

## Refernces
- (https://www.thingiverse.com/thing:3465651)
- https://hypertriangle.com/~alex/delta-robot-tutorial/
 

