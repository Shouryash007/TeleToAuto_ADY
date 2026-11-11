>> NOTE: BEFORE STARTING THE SERVER OR ROBOT MAKE SURE YOU ARE INSIDE THE ENVIRONMENT ( source venv/bin/activate)
cloudflare: cloudflared tunnel --url http://127.0.0.1:8000
server: uvicorn server.server:app --host 127.0.0.1 --port 8000
robot: python file_name.py

>> Credentials
Username: operator1
Password: user1@123
Username: operator2
Password: user2@123

>> To create another robot things needed to change:
1.robot_id
2.robot_token
3.Signaling_server
4.camera_device
in json add the class of that roboot.

>> to launch the so101 handtrack dual hand:
ros2 launch ./so101_dual_gazebo.launch.py urdf_file:=$PWD/so101_new_calib.urdf camera_index:=1 2>&1 | tee launch.log


>> implementable-things Approval-needed:
1. For user-robot sync include motion scaling like user moves little and robot moves a increment on it 1:1.5 sort of.
2. Decide on whether React is necessary for the frontend part of the website Laptop/mobile.
3. For the vision based hand track rather than a normal rgb camera using a stereo camera as a start phase(for individual arm we can go with seperate stereo) and later on the further path we can go with depth based tracking.
4. For a second approach we could gowith marker based tracking like AprilTags, ArUco marker, Reflective motion-capture markers or combine the hand track with these marker based tracking as we are using hand held controllers so imu will be able o easily install.
5. Using a cloud storage where we can store/log cam data session recordings that would highly help in debugging.
6. [Future] predictive state estimation based EKF filtering for data transfer

>> Group-Work:
1. Research on the india based companies that closely align with our products and also on products indie/international level that closely align with our prototype.
2. Inform on the designing part for stereo fixing on the VR and hand-held controllers.
3. Circuit design for the hand-held controllers.

>> things need to be implemented in hardware-stage:
1. creating a udev rules for camera file where the file identifies the camera connected applies the appopriate permissions and allows the application to acess the device.
2. Camera driver file is needed when we move on to controller based or processor in robot like rasberry-pi.
