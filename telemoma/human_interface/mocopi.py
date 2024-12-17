from telemoma.human_interface.teleop_core import BaseTeleopInterface, TeleopObservation, TeleopAction
import socket
import struct
from human_interface.runner import Runner
from telemoma.utils.general_utils import run_threaded_command
import numpy as np
import time
import queue


class MocopiTeleopInterface(BaseTeleopInterface):
    """
    Teleoperation interface for Mocopi motion capture system.
    """

def is_field(name):
    return name.isalpha()

def _deserialize(data, index, length, is_list=False):
    result = [] if is_list else {}
    end_pos = index + length
    while end_pos - index > 8 and is_field(data[index + 4:index + 8]):
        size = struct.unpack("@i", data[index: index + 4])[0]
        index += 4
        field = data[index:index + 4]
        index += 4
        value, index2 = _deserialize(data, index, size, field in [b"btrs", b"bons"])
        index = index2
        if is_list:
            result.append(value)
        else:
            result[field.decode()] = value
    if len(result) == 0:
        body = data[index:index + length]
        return body, index + len(body)
    else:
        return result, index

def _process_packet(message):
    data = _deserialize(message, 0, len(message), False)[0]
    data["head"]["ftyp"] = data["head"]["ftyp"].decode()
    data["head"]["vrsn"] = ord(data["head"]["vrsn"])
    data["sndf"]["ipad"] = struct.unpack("@BBBBBBBB", data["sndf"]["ipad"])
    data["sndf"]["rcvp"] = struct.unpack("@H", data["sndf"]["rcvp"])[0]
    if "skdf" in data:
        for item in data["skdf"]["bons"]:
            item["bnid"] = struct.unpack("@H", item["bnid"])[0]
            item["pbid"] = struct.unpack("@H", item["pbid"])[0]
            item["tran"] = struct.unpack("@fffffff", item["tran"])
    elif "fram" in data:
        data["fram"]["fnum"] = struct.unpack("@I", data["fram"]["fnum"])[0]
        data["fram"]["time"] = struct.unpack("@I", data["fram"]["time"])[0]
        for item in data["fram"]["btrs"]:
            item["bnid"] = struct.unpack("@H", item["bnid"])[0]
            item["tran"] = struct.unpack("@fffffff", item["tran"])
    return data


class Receiver(Runner):
    def __init__(self, addr="192.168.0.235", port=12351):
        self.addr = addr
        self.port = port
        self.queue = queue.Queue()  # Initialize the queue in the Receiver class

    def loop(self):
        self.socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.socket.bind((self.addr, self.port))
        while True:
            try:
                message, client_addr = self.socket.recvfrom(2048)
                data = _process_packet(message)
                self.queue.put(data)  # Put data into the queue
                # print(data)
                # Print the pose data in real-time
                # if "fram" in data:
                #     print("Frame Number:", data["fram"]["fnum"])
                #     print("Time:", data["fram"]["time"])
                #     for i, item in enumerate(data["fram"]["btrs"]):
                #         print(f"Bone {i+1} (ID: {item['bnid']}):")
                #         print(f"  Translation: ({item['tran'][0]}, {item['tran'][1]}, {item['tran'][2]})")
                #         print(f"  Rotation: ({item['tran'][3]}, {item['tran'][4]}, {item['tran'][5]}, {item['tran'][6]})")
                #     print()
            except KeyError as e:
                print(f"KeyError: {e}")

    def start(self):
        # Start the receiver loop in a new thread
        # Note: The loop method should be run in a separate thread or process
        Receiver().run(self.queue)

    def stop(self):
        # Stop the receiver
        print("Receiver stopping...")
        self.socket.close()


class MocopiTeleopInterface(BaseTeleopInterface):
    def __init__(self, addr="192.168.0.235", port=12351, *args, **kwargs):
        """
        Initializes the teleop policy
        """
        super().__init__(*args, **kwargs)
        self.receiver = Receiver(addr=addr, port=port)  # Initialize receiver with the queue
        self.latest_pose = None  # ToDo: store the latest pose data
        self.raw_data = {}

        self.plot = []

    def start(self) -> None:
        """
        Start the mcp-receiver and the pose update thread.
        """
        self.receiver.start()  # Start the receiver
        run_threaded_command(self._update_internal_state)  # Start updating the pose data
        self.set_reference()

    def stop(self) -> None:
        """
        Stops the teleop policy by stopping the receiver.
        """
        print("Stopping Mocopi interface...")
        self.receiver.stop()

    def reset_state(self) -> None:
        """
        Reset all internal states.
        """
        self.latest_pose = None  # Reset pose data
        self.raw_data = {}

    def set_reference(self, timer=2):
        start = time.time()
        self.reference_pose = None
        while time.time() -start < timer:
            self.reference_pose = self.latest_pose
            print('Setting Referene...', time.time())
        # self.reference_queue = deque(maxlen=15)
        # self.reference_queue.append(self.reference_pose)

    def _update_internal_state(self) -> None:
        """
        Update self.raw_data with the latest pose data from the receiver.
        This method runs in a loop to continuously fetch the latest pose data.
        """
        while True:
            try:
                # Get the latest pose data from the receiver
                if not self.receiver.queue.empty():
                    # print(self.receiver.queue.get())
                    self.raw_data = self.receiver.queue.get()
                    self.latest_pose = {item["bnid"]: item["tran"] for item in self.raw_data["fram"]["btrs"]}
                    # self.raw_data = self.latest_pose  # Update raw_data with the latest pose
                    # print(latest_pose)
                time.sleep(0.01)  # Adjust as needed for performance
                # print(self.receiver.queue.get())
            except Exception as e:
                print(f"Error updating pose: {e}")

    def get_action(self, obs: TeleopObservation) -> TeleopAction:
        """
        Get the action of a body part based on the latest pose data.
        """
        action = self.get_default_action()
        if self.latest_pose is None:
            return action  # Return default action if no pose data

        if len(self.plot) == 100:
            import matplotlib.pyplot as plt

            plot = np.array(self.plot)
            plt.figure()
            plt.plot(plot[:, 0], label='x')
            plt.plot(plot[:, 1], label='y')
            plt.plot(plot[:, 2], label='z')
            plt.legend()
            plt.show()


        # Process the latest pose data to determine actions
        action.base, action.torso, action.left, action.right = self.process_pose_data(self.latest_pose, obs)
        return action

    def process_pose_data(self, pose_data, obs: TeleopObservation):
        """
        Process the pose data to extract actions for the robot.
        """
        base_action = np.zeros(3)  # Placeholder for base movement
        torso_action = np.zeros(1)  # Placeholder for torso movement
        left_hand_action = np.zeros(7)  # Placeholder for left hand action
        right_hand_action = np.zeros(7)  # Placeholder for right hand action

        # Extract translation and rotation for each bone\
        #convert pose_data to list/dictionary where indexing with bone id will give translation
        # Example mapping for specific bones (you'll need to adapt this)
        # Example for the root bone (base)
        from telemoma.utils.transformations import quat_to_euler, euler_to_quat, quat_diff 
        translation = pose_data[0]
        
        base_delta_euler = np.zeros(3)
        base_delta_euler[2] = obs.base[2]
        base_delta_quat = euler_to_quat(base_delta_euler)
        target_delta_quat = translation[3:7]#quat_diff(translation[3:7], self.reference_pose[0][3:7])
        quat_action = quat_diff(target_delta_quat, base_delta_quat)
        euler_action = quat_to_euler(quat_action)

        action = np.array([0, 0, euler_action[2]])
        base_action = action.clip(-1, 1)
        base_action = np.array(translation[:2])
        elif bone_id == 14:  # Example for left hand
        left_hand_action[:3] = translation[:3]  # Position
        left_hand_action[3:7] = translation[3:7]  # Rotation (quaternion)| Convert to Euler angle and pass in as 3rd part of base action

        elif bone_id == 18:  # Example for right hand
        right_hand_action[:3] = translation[:3]  # Position
        right_hand_action[3:7] = translation[3:7]  # Rotation (quaternion)
        
        elif bone_id == 4:
        torso_action = translation[:3]
    
        # print (base_action)
        print(quat_to_euler(target_delta_quat))
        self.plot.append(quat_to_euler(target_delta_quat))
        return base_action, torso_action, left_hand_action, right_hand_action
