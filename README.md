 Project Directory Structure and File Descriptions

 1. `.github/`
Contains GitHub-related configuration files, including repository workflows and automation settings.

---

 2. `client/`
Contains the frontend application responsible for the operator interface, hand tracking, inverse kinematics, mobile orientation and robot communication.

    2.1 `app.js`
    Main frontend application file. Coordinates the hand-tracking system, inverse kinematics controller, robot connection and user interface.

    2.2 `digital-twin.js`
    Handles the digital-twin functionality, providing the interface between the simulated robot representation and the teleoperation application.

    2.3 `dual-hand-ik-controller.js`
    Implements the main dual-hand inverse kinematics system. Processes MediaPipe hand landmarks and calculates corresponding robotic arm joint targets using inverse kinematics.

    2.4 `hand-tracker.js`
    Initializes the webcam and MediaPipe Hands model, detects both hands, extracts 21 landmarks per hand and forwards the landmark data to the inverse kinematics controller.

    2.5 `index.html`
    Main operator webpage containing the teleoperation dashboard and application layout.

    2.6 `manipulator-controller.js`
    Contains the manipulator joint-mapping and gripper-control logic from the earlier controller implementation.

    2.7 `mobile-gyro.js`
    Handles mobile phone orientation sensing and transmits orientation data for remote camera movement.

    2.8 `mobile-index.html`
    Webpage used to access the mobile orientation interface and provide phone sensor data.

    2.9 `robot-connection.js`
    Manages WebSocket signaling, WebRTC connections and DataChannels for transmitting joint targets and orientation data between the operator and robot.

    2.10 `style.css`
    Contains the frontend styling, layout, colours and responsive design of the web application.

    2.11 `ui-controller.js`
    Manages the operator interface, including login-related UI, robot selection, tracking controls, connection status and live joint-value displays.

---

 3. `Credentials/`
Contains configuration and authentication-related files used by the application.

    3.1 `Password_generator.py`
    Python script for generating passwords.

    3.2 `robots.json`
    Stores robot configuration and robot-related information used by the server.

    3.3 `users.json`
    Stores user account information used by the authentication system.

---

 4. `Others/`
Contains additional hardware-control files and reference configurations that are not part of the primary teleoperation execution flow.

    4.1 `arduino_code.cpp`
    Arduino firmware for interfacing with and controlling connected hardware.

    4.2 `follower_before.json`
    Reference configuration containing the follower manipulator's earlier calibration values.

    4.3 `follower_controller.py`
    Python controller responsible for handling follower manipulator control.

    4.4 `follower_driver.py`
    Driver interface for communicating with the follower manipulator hardware.

    4.5 `leader_before.json`
    Reference configuration containing the leader manipulator's earlier calibration values.

---

 5. `robot/`
Contains the robot-side Python programs responsible for establishing communication with the server and handling remote robot operations.

    5.1 `robot1.py`
    Robot-side communication program responsible for connecting to the teleoperation server, establishing WebRTC communication and processing incoming control commands.

    5.2 `robot2.py`
    Second robot-side communication program for operating another robot instance.

---

 6. `server/`
Contains the backend application responsible for managing operator authentication, robot connections and communication signaling.

    6.1 `server.py`
    Main backend server. Handles user authentication, robot registration, operator sessions, access management and WebSocket signaling for WebRTC connections.

---

 7. `static/`
Contains simulation resources, Gazebo integration files, robot models and ROS 2 bridge implementations.

    7.1 `gyro/`
    Contains the scripts responsible for mobile orientation processing and simulated robot camera video streaming.

        7.1.1 `camera_gyro_bridges.launch.py`
        ROS 2 launch file that starts the Gazebo camera bridge and mobile orientation bridge.

        7.1.2 `gazebo_camera_bridge.py`
        Receives camera images from the Gazebo simulation and forwards them for remote viewing through the teleoperation interface.

        7.1.3 `gyro_to_gazebo.py`
        Receives mobile phone orientation data over UDP and publishes the corresponding camera yaw and vertical tilt commands to Gazebo controllers.

    7.2 `meshes/`
    Contains 3D mesh assets used by the robot model for visualization and simulation.

    7.3 `urdf/`
    Contains the robot's Unified Robot Description Format (URDF) model files.

        7.3.1 `humanoid_gazebo.urdf`
        Defines the humanoid robot's links, joints, physical properties and visual/collision geometry for Gazebo simulation.

    7.4 `arm_bridge.py`
    Receives arm joint targets over UDP and publishes the corresponding left and right arm commands to ROS 2 Gazebo position controllers.

    7.5 `gazebo_humanoid.launch.py`
    Launches the humanoid robot simulation in Gazebo.

    7.6 `humanoid_controllers.yaml`
    Defines the ROS 2 controllers, joint configurations and controller parameters used to control the simulated humanoid robot.

    7.7 `so101_follower.zip`
    Compressed archive containing SO101 follower-related files or resources.

---

 8. `.venv/`
Contains the project's local Python virtual environment, including its installed dependencies and packages.

 9. `.env.example`
Example environment configuration file containing the environment variables required to configure the application.

 10. `.gitignore`
Specifies files and directories that Git should exclude from version control.

 11. `cloudflared-linux-amd64.deb`
Debian installation package for Cloudflare Tunnel, used to expose the locally hosted application through a public endpoint.

 12. `README.md`
Main project documentation containing the project overview, setup instructions, dependencies and execution procedures.

 13. `requirements.txt`
Lists the Python dependencies required to run the backend, robot communication and simulation-related Python programs.