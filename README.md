# EECS 373: Proposal for Team 16 Self-Balancing Robot Project 
**Authors:** Yanlei Chen, Robert Conforti, Reshob Islam, Sushovit Khadka 

## Section 1: High Level Description 

Our project aims to design an autonomous, two-wheeled robotic platform utilizing an inverted pendulum control strategy.  The system is designed for high-precision dynamic balancing and intelligent target tracking.  By integrating a vision-based perception system with redundant ultrasonic obstacle avoidance, the robot achieves robust performance in unstructured environments while maintaining a vertical balance state through real-time feedback control.  The system utilizes real-time orientation data to maintain balance while simultaneously executing high-level autonomous tasks, such as object tracking and remote-controlled navigation. 

The central processing unit, the STM32 Nucleo-L4R5ZI-P, orchestrates a hierarchical control loop.  To maximize reliability, the architecture utilizes Sensor Fusion to prioritize stability and safety: 
* **Balancing Engine:** A BNO055 IMU provides 6-axis spatial orientation, enabling the PID controller to counteract gravitational torque at high frequencies (100Hz+). 
* **Perception and Guidance:** A Pixy2 vision sensor performs onboard feature extraction to identify and track specific color signatures. 
* **Safety Layer:** A Time-of-Flight (VL53L0X) or Ultrasonic (HC-SR04) sensor provides continuous distance mapping.  This acts as a hardware-level safety override that preempts tracking commands if an obstacle is detected within a defined critical radius. 
* **Power and Actuation:** A 12V LiPo battery feeds a power distribution system, providing current to the motors via an L298 H-Bridge motor driver, which translates PWM signals from the Nucleo into precise wheel motion. 
* **Human-Machine Interface:** A wireless PS2 DualShock controller facilitates manual overrides, while an onboard 0.96" OLED display provides real-time telemetry and system status updates. 

The robot’s intelligence is governed by a State-Driven Navigation Algorithm, which prioritizes inputs based on safety, human intent, and autonomous goals: 
* **Manual Override:** Through the PS2 wireless controller, the user can manually command the robot's heading and velocity.  When active, this state overrides the Pixy2 tracking data, allowing for direct pilot control while the PID loop continues to maintain the robot’s upright balance. 
* **PID Stabilization:** The fundamental control loop runs at a high-frequency (100Hz+) cycle.  It calculates the necessary PWM signals for the L298 H-Bridge to counteract gravity, ensuring the robot remains upright regardless of the active navigation state. 
* **Autonomous Tracking:** When the Manual Override is disengaged, the Pixy2 transmits real-time vector coordinates to the Nucleo.  The processor treats these coordinates as a dynamic setpoint offset, steering the robot to follow the target object. 
* **Obstacle Avoidance:** Regardless of whether the robot is in Manual or Autonomous mode, the distance sensor (VL53L0X) continuously monitors the path.  If an obstacle is detected within a defined threshold, the logic initiates a safety interrupt, prioritizing collision prevention over all other commands. 

The following table: 

| State | Priority | Source | Behavior |
| :--- | :--- | :--- | :--- |
| Emergency Stop | 1 (Immediate) | Distance Sensor | Force-stop to prevent collision |
| Manual Override | 2 | PS2 Controller | Pilot steers robot PID maintains balance |
| Autonomous Track | 3 | Pixy2 Camera | Robot tracks target object |
| Stationary | 4 | Default | Maintain vertical equilibrium at origin |

1. **Stabilization Function:** The system executes a high-frequency PID control loop to process orientation data from the BNO055 IMU.  By modulating PWM signals to the motors, the robot dynamically shifts its base to maintain its vertical center of gravity against gravitational torque. 
2. **Autonomous Navigation Function:** The Pixy2 camera processes visual data to identify target color signatures and transmit coordinate vectors to the Nucleo.  The controller then translates these vectors into real-time steering adjustments, enabling the robot to track and follow the designated object. 
3. **Safety and Obstacle Avoidance Function:** The robot continuously polls distance data via ultrasonic or Time-of-Flight sensors to map the immediate environment.  If an obstacle is detected within a predefined critical radius, the system triggers a high-priority interrupt to halt movement and prevent collisions. 
4. **Human-Machine Interaction Function:** The system decodes wireless input from the PS2 controller to allow for manual pilot intervention during operation.  A software-based state machine manages seamless transitions between manual steering and autonomous tracking, preventing erratic behavior during mode switching. 
5. **Power Management Function:** The power distribution board regulates the 12V battery source into stable 5V and 3.3V rails to ensure clean power for sensitive electronics.  This subsystem also integrates a hardware-level safety kill switch to immediately terminate power to the motors during an emergency. 
6. **System Status and Telemetry Function:** The onboard OLED display provides real-time feedback by visualizing system telemetry, such as current operating mode (Manual vs. Autonomous), target lock status, and battery levels.  This visual interface enables the team to monitor internal PID values and system health during testing without requiring a constant serial connection to a laptop. 

### Inputs (Sensors and Interfaces) 
* **Physical Orientation (BNO055):** Provides the high-speed angular velocity and acceleration data necessary for the PID control loop. 
* **Vision Data (Pixy2):** Supplies the (x, y) location of the target object, allowing the robot to calculate the error relative to the center of its field of view. 
* **Distance Mapping (VL53L0X/HC-SR04):** Constantly monitors the environment for obstacles within a defined critical safety radius. 
* **Pilot Command (PS2 Controller):** Provides manual thrust and yaw inputs via the wireless receiver to override autonomous behaviors. 

### Outputs (Actuators and Status) 
* **Motor Command (PWM):** The Nucleo calculates the required motor speed and direction and sends a Pulse Width Modulation (PWM) signal to the L298 H-Bridge.  This is the physical "force" that keeps the robot balanced. 
* **Visual Status (OLED Display):** Provides real-time feedback on the robot’s current state (e.g., MANUAL, TRACKING, OBSTACLE DETECTED). 
* **System Alerts (Buzzer):** Optional auditory feedback for state changes or emergency stops. 

## Section 2: Functional Diagram 

*(Diagram included in original PDF mapping the microcontroller to the Pixy2 Camera, BNO055 IMU, VL53L0X Distance Sensor, PS2 Wireless Receiver, L298 H-Bridge, and Power Distribution Board)*

## Section 3: Component Diagram 

*(Diagram included in original PDF detailing the signal, data, and power lines between the Flight Controller, Radio Receiver, Camera, Distance Sensor, PDB, and Motors)*

## Section 4: Component List 

### Stock Components 

#### Main Function / Processing 
* **STM32 Nucleo L4R5ZI-P board:** Main Microcontroller 
* **BNO055 Accel, Gyro, and Magnetometer:** Orientation for balancing 
* **Pixy Camera:** Computer vision for object tracking 
* **VL53L0X Time-of-Flight Sensor OR HC-S404 Ultrasonic Sensor (with headers):** Distance detection for obstacle avoidance [cite: 114, 115]
* **Monochrome 0.96" 128x64 OLED (I2C):** Onboard status display 

#### Actuation and Drive 
* **2x Geared DC Motors:** Thrust and balancing output 
* **L298 H-Bridge Heavy Duty (with breakout):** Motor driver 

#### Wireless Communication and Debugging 
* **PS2 DualShock 2 Wireless Controller:** Manual pilot override 
* **FTDI Micro USB to Serial (3.3V):** Helps debug camera/sensor signals on a laptop before sending to the robot 

#### Power System 
* **33.3Wh 12V + 5V (USB) Battery Pack:** Main power source 
* **LDO 5V Regulator LM2940 + Caps:** Ensures clean power delivery to sensitive sensors 

---

### Non-Stock Components 

* **Custom 3D-Printed Chassis:** Designed specifically as a tall "inverted pendulum" rather than a flat turtle chassis 
* **2x High-Grip Wheels:** Attaches to the geared DC motors (standard hard plastic wheels will slip when trying to balance) 
* **Power Distribution Board (PDB) or Custom Perfboard:** Safely routes the 12V battery to the L298 and steps down power to the Nucleo 
* **Wiring Harness, Jumper Wires, and XT60/Barrel Connectors:** For safe battery connections 
* **Mechanical Fasteners:** Screws, nuts, brass standoffs for securely mounting boards 
