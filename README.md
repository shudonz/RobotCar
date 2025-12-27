# RobotCar - Arduino 
This code will run on a KS0470 4WD Robot Car Kit from Keyestudio.

This code is an Arduino sketch for a robot car that incorporates multiple subsystems, including motor control, an ultrasonic sensor, an IR remote control, a servo motor, and an LED matrix display. Below is a full conceptual overview of how the code works and what each part of it does:

1. **Purpose and Components Controlled**  
   The robot is designed to:

   - Respond to user commands via an IR remote control.  
   - Operate in either manual mode (direct control via IR commands) or autonomous mode.  
   - Avoid obstacles using an ultrasonic distance sensor.  
   - Turn towards open paths using a servo motor and a scanning mechanism.  
   - Display a scrolling animation (text "CLANKER") on an LED matrix.  

   It integrates several hardware modules:  
   - **Motors**: Provide motion to the robot (forward, backward, left, and right).  
   - **Ultrasonic sensor (SR04)**: Measure distance to obstacles in front of the robot.  
   - **IR remote control**: Allows for manual operation via an infrared controller.  
   - **Servo motor**: Allows movement of the ultrasonic sensor to scan the environment left-to-right.  
   - **LED Matrix**: Displays scrolling text for aesthetic or informational purposes.  

2. **Hardware Initialization**  
   Several hardware subsystems are initialized in the `setup()` function:  
   - Motor control pins are set as outputs for controlling direction and speed of the wheels.  
   - IR receiver is enabled to start listening for commands.  
   - Servo motor is attached and set to its initial (centered) position at 90 degrees.  
   - LED Matrix is initialized, and scrolling text ("CLANKER") is prepared.  
   - The ultrasonic sensor is initialized with the appropriate TRIG and ECHO pins.  

3. **Control Modes**  
   The car has two modes of operation:  
     - **Manual Mode (when autoMode is false):** Directly driven by the user using the IR remote.  
         - The remote sends commands to move forward, backward, left, or right, as well as stop.  
         - The robot checks if there is any obstacle in front (via the ultrasonic sensor) and halts forward motion if necessary to prevent a collision.  
     - **Autonomous Mode (when autoMode is true):** The robot behaves autonomously, navigating around obstacles and avoiding collisions.  
         - Uses a state machine (AutoState) to define its behavior:  
             - **IDLE:** The robot waits for clear space ahead to move forward or starts backing away from obstacles.  
             - **MOVE_FORWARD:** Moves forward while continuously checking for obstacles and potential stalls.  
             - **MOVE_BACK:** Moves backward briefly to create space after encountering an obstacle.  
             - **SCAN:** Turns the servo to scan for the clearest path (i.e., the direction with the largest distance from obstacles).  
             - **TURN_TO_CLEAR:** Rotates toward the clearest path before resuming forward motion.  

4. **Obstacle Avoidance**  
   - The ultrasonic sensor (SR04) is used to measure distances to obstacles.  
   - A median filter is applied to the distance readings to remove noise and calculate a stable measurement.  
   - If the distance ahead is less than 30 cm:  
     - In manual mode, forward motion is stopped to prevent collisions.  
     - In autonomous mode, the robot determines the next best path.  

5. **LED Matrix Scrolling Text**  
   - The LED matrix displays a scrolling "CLANKER" text.  
   - Text is scrolled by continuously updating a buffer that shifts the displayed pixels horizontally.  
   - Custom patterns for the letters C, L, A, N, K, E, and R are defined as 8x8 arrays of bits.  
   - This functionality runs in parallel to other subsystems, ensuring consistent scrolling during operation.  

6. **Stall Detection**  
   Used in autonomous mode to detect when the robot gets stuck.  
   - Monitors distance changes while moving forward:  
     - If no significant distance change is observed over time (<4 cm), the robot is considered stalled.  
     - If the stall persists beyond a timeout (800 ms), the robot transitions to the MOVE_BACK state to create space and retry.  

7. **IR Remote Control Handling**  
   - The robot decodes and acts on commands received via the IR remote. The mappings for IR signals are:  
     - `0xFF629D`: Move forward.  
     - `0xFFA857`: Move backward.  
     - `0xFF22DD`: Turn left.  
     - `0xFFC23D`: Turn right.  
     - `0xFF02FD`: Stop and hold position.  
     - `0xFF6897`: Toggle between manual and autonomous modes.  
   - The `driveCmd` variable holds the current command, and actions are taken accordingly.

8. **State Machine (Autonomous Mode)**  
   The system uses an enum-based finite state machine for structured behavior in autonomous mode:  
   - **IDLE:** Waits for clear space or backs away from obstacles.  
   - **MOVE_FORWARD:** Moves straight ahead, checking for new obstacles or stalls.  
   - **MOVE_BACK:** Moves backward briefly before transitioning to scanning.  
   - **SCAN:** Tests multiple angles with the servo to determine the best (clearest) direction.  
   - **TURN_TO_CLEAR:** Rotates the servo/motors to face the chosen path.  

9. **Motor and Servo Handling**  
   - The robot uses helper functions for motor and servo operations:  
     - `front()`, `back()`, `left()`, and `right()` manage motor direction and speed.  
     - `Stop()` stops all wheel motion.  
     - `myServo.write()` changes the angle of the servo (used during scanning).  

10. **Performance Optimizations**  
    - **Median Distance Calculation:** Uses a bubble sort to sort distance readings and selects the median value to ensure robust distance measurements.  
    - **Debounced Scrolling:** Limits matrix updates to every 100 ms to reduce resource usage.  
    - **Non-blocking Structure:** Allows the different subsystems (movement, scrolling text, remote commands) to run independently and responsively.

---

### Summary of Behavior

The robot's functionality can be summarized as:

- **Manual Mode:** User-controlled via IR remote. Obstacle avoidance applies to forward motion only.  
- **Autonomous Mode:** Self-navigating and obstacle-avoiding, leveraging the ultrasonic sensor and servo motor to make intelligent decisions.  
- **Aesthetic LED Display:** Continuously scrolls the "CLANKER" text during operation.  
- **Failure Handling:** Detects stalls and retries alternate paths when blocked.
