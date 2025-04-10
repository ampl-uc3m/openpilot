#!/usr/bin/env python3
import os
import argparse
import threading
import json
import numpy as np
import zmq

from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import HARDWARE

class ZMQController:
    def __init__(self):
        # Setup ZeroMQ subscriber
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.connect("tcp://localhost:5557")  # Connect to the bridge
        self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "ros_control")

        # Initialize values
        self.axes_values = {'throttle': 0., 'brake': 0., 'steer': 0.}
        self.axes_order = ['throttle_brake', 'steer']  # Combined throttle/brake as one axis
        self.cancel = False
        self.last_update = 0

        # Debug variables to track changes
        self.last_brake_value = 0.0
        self.last_throttle_value = 0.0

    def get_combined_throttle_brake(self):
        """
        CORRECTED axis mapping according to specified behavior:
        - If throttle=1.0 → return +1.0 (full throttle)
        - If brake=1.0 → return -1.0 (full brake)
        - If both are at zero, they cancel → return 0.0 (neutral)
        """
        # Print debug info when values change
        if self.axes_values['brake'] != self.last_brake_value or self.axes_values['throttle'] != self.last_throttle_value:
           # print(f"DEBUG: Processed brake={self.axes_values['brake']}, throttle={self.axes_values['throttle']}")
            self.last_brake_value = self.axes_values['brake']
            self.last_throttle_value = self.axes_values['throttle']

        # Calculate combined value (throttle is positive, brake is negative)
        combined = float(self.axes_values['throttle'] - self.axes_values['brake'])

        # Ensure within bounds
        combined = float(np.clip(combined, -1.0, 1.0))

        return combined

    def update(self):
        try:
            # Check for new message with 10ms timeout
            if self.zmq_socket.poll(10):
                message = self.zmq_socket.recv_string()
                topic, data_str = message.split(" ", 1)
                data = json.loads(data_str)

                # Update steer value as normal
                self.axes_values['steer'] = float(np.clip(float(data['steer']), -1.0, 1.0))

                # REMAP: Convert brake from trigger range (1 to -1) to standard range (0 to 1)
                # Where 1 (not pressed) → 0 (no brake) and -1 (fully pressed) → 1 (full brake)
                raw_brake = float(data['brake'])
                remapped_brake = float((1 - raw_brake) / 2)  # Linear remapping
                self.axes_values['brake'] = remapped_brake

                # REMAP: Convert throttle from trigger range (1 to -1) to standard range (0 to 1)
                # Where 1 (not pressed) → 0 (no throttle) and -1 (fully pressed) → 1 (full throttle)
                raw_throttle = float(data['throttle'])
                remapped_throttle = float((1 - raw_throttle) / 2)  # Linear remapping
                self.axes_values['throttle'] = remapped_throttle

                self.last_update = data['timestamp']

                # Show raw and remapped values when they change significantly
                # if abs(raw_brake - 1.0) > 0.05 or abs(raw_throttle - 1.0) > 0.05:
                  #  print(f"REMAP: brake {raw_brake:.2f}→{remapped_brake:.2f}, throttle {raw_throttle:.2f}→{remapped_throttle:.2f}")


                return True
        except Exception as e:
            print(f"Error receiving ZMQ message: {e}")
        return False

def send_thread(controller):
    pm = messaging.PubMaster(['testJoystick'])
    rk = Ratekeeper(100, print_delay_threshold=None)

    while True:
        # Calculate the combined throttle/brake value
        throttle_brake = controller.get_combined_throttle_brake()
        steer = controller.axes_values['steer']

        # Display more frequently (every 10 frames = 10Hz)
        if rk.frame % 10 == 0:
            # Modified to make it clearer what's being sent
            print(f'SENDING TO OPENPILOT: throttle_brake={round(throttle_brake, 3)}, steer={round(steer, 3)}')

        joystick_msg = messaging.new_message('testJoystick')
        joystick_msg.valid = True

        # Set axes in the order expected by openpilot
        # Explicitly convert to native Python float to avoid numpy.float64 issues
        joystick_msg.testJoystick.axes = [
            float(throttle_brake),  # Using the pre-calculated value to ensure consistency
            float(steer)            # Explicit conversion to Python float
        ]

        pm.send('testJoystick', joystick_msg)
        rk.keep_time()

def zmq_control_thread(controller):
    Params().put_bool('JoystickDebugMode', True)
    threading.Thread(target=send_thread, args=(controller,), daemon=True).start()

    # Keep checking for updates
    while True:
        controller.update()

def main():
    controller = ZMQController()
    zmq_control_thread(controller)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Reads control values from ZMQ bridge and sends them to openpilot.',
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    args = parser.parse_args()

    print()
    print('Using ZMQ bridge for control from ROS2 topics (with trigger remapping):')
    print('- Steering: /steer_pub (Float64 -1.0 to 1.0)')
    print('- Braking: /brake_pub (Float64 1.0 to -1.0) → remapped to 0.0 to 1.0')
    print('- Acceleration: /accel_pub (Float64 1.0 to -1.0) → remapped to 0.0 to 1.0')
    print('Trigger behavior:')
    print('- Trigger not pressed = 1.0 (raw) → 0.0 (remapped)')
    print('- Trigger half pressed = 0.0 (raw) → 0.5 (remapped)')
    print('- Trigger fully pressed = -1.0 (raw) → 1.0 (remapped)')

    main()
