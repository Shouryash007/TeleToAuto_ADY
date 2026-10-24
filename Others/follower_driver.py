import json

from scservo_sdk import PortHandler, PacketHandler

BAUDRATE = 1000000
PROTOCOL = 0

ADDR_GOAL_POSITION = 42


class FollowerDriver:

    def __init__(self,
                 port="/dev/ttyACM1",
                 calibration_file="calibration.json"):

        self.port = PortHandler(port)
        self.packet = PacketHandler(PROTOCOL)

        with open(calibration_file, "r") as f:
            self.calibration = json.load(f)

    def connect(self):

        if not self.port.openPort():
            raise RuntimeError("Failed to open port")

        if not self.port.setBaudRate(BAUDRATE):
            raise RuntimeError("Failed to set baudrate")

        print("Follower connected.")

    def disconnect(self):
        self.port.closePort()

    def degree_to_encoder(self, joint_name, degrees):

        joint = self.calibration[joint_name]

        minimum = joint["range_min"]
        maximum = joint["range_max"]

        midpoint = (minimum + maximum) / 2

        encoder = int((degrees * 4095 / 360) + midpoint)

        encoder = max(minimum, min(maximum, encoder))

        return encoder

    def send_action(self, action):

        for key, degrees in action.items():

            joint = key.replace(".pos", "")

            servo_id = self.calibration[joint]["id"]

            encoder = self.degree_to_encoder(joint, degrees)

            self.packet.write2ByteTxRx(
                self.port,
                servo_id,
                ADDR_GOAL_POSITION,
                encoder
            )
