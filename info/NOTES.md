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

>> Things to do:
1. As of now the joint targets are directly transfered to robot instead of going through the server so we need to fix that .

2. TURN credentials sit in page globals.
window.TELEOP_TURN_USERNAME and window.TELEOP_TURN_CREDENTIAL are readable by anyone who can load the page. If they are long-lived, anyone can use your TURN server as a free relay. Have the server issue short-lived credentials after login, using the TURN REST scheme with an expiry.

3. Browser and phone messages are forwarded without filtering.
_forward_from_browser sends any message straight to the robot. Your browser only ever sends the SDP answer, so allow just that:

python
ALLOWED_BROWSER_TYPES = {"answer"}

async def _forward_from_browser(robot_id, data, raw):
    if data.get("type") not in ALLOWED_BROWSER_TYPES or len(raw) > 16_384:
        return
    ...

For the phone’s answer and candidate, build a new dict with only the expected fields instead of forwarding data with an added key.

4. The first message has no timeout.
await websocket.receive_text() waits forever, so unauthenticated connections can pile up. Add a timeout:

python
data = json.loads(await asyncio.wait_for(websocket.receive_text(), timeout=10))

Also cap message size on the server (uvicorn has --ws-max-size) and add a small rate limit in the message loop.

5. Tokens are checked once.
If a session expires or access is revoked, an open socket stays authorised. Re-check the token every minute or so, or close the socket at token expiry.

5. No brute-force protection.
Add attempt limits per IP and per account on both robot and browser logins.

6. Plain ws: is possible.
signalingUrl() falls back to ws: when the page isn’t HTTPS. Your token travels in the first message, so require wss: everywhere outside local testing. Adding a WebSocket Origin check is cheap as well.

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

>> server folder:
server/
│
├── main.py                  # Starts FastAPI
├── config.py                # Paths, settings, environment variables
├── auth.py                  # Login and authentication
├── robot_manager.py         # Robot connections and availability
├── routes.py                # HTTP API endpoints
├── websocket.py             # WebSocket connections and message routing
