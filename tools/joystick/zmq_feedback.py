#!/usr/bin/env python3
import math
import zmq
import time
import json
import os
from enum import Enum  # Importación del módulo Enum

from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, DT_CTRL
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from cereal import messaging, car
from opendbc.car import structs

def get_nested_attr(obj, attr):
    """Get nested attributes from an object using dot notation."""
    attrs = attr.split('.')
    for a in attrs:
        obj = getattr(obj, a)
    return obj

def load_config():
    """Load configuration from JSON file."""
    script_dir = os.path.dirname(os.path.realpath(__file__))
    config_path = os.path.join(script_dir, 'zmq_feedback_config.json')

    with open(config_path, 'r') as f:
        return json.load(f)

def serialize_data(data):
    """Helper function to convert non-serializable types to serializable."""
    for key, value in data.items():
        # If the value has an __int__ method (like enums or other special types)
        if hasattr(value, '__int__'):
            data[key] = int(value)
        elif isinstance(value, bool):
            data[key] = int(value)  # Convert boolean to int
        elif isinstance(value, float):
            data[key] = round(value, 3)  # Round float values
        elif isinstance(value, Enum):  # Handle enums
            data[key] = str(value)  # Convert enum to string
        elif hasattr(value, '__str__'):  # Check if the value has a string representation
            data[key] = str(value)  # Convert the object to its string representation
    return data

def main():
    # Load configuration
    config = load_config()

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5560")  # port for publishing data

    print("1")

    params = Params()
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
    print("CP:", CP)
    VM = VehicleModel(CP)
    print("2")

    sm = messaging.SubMaster(['carState', 'liveParameters'], frequency=1. / DT_CTRL)
    rk = Ratekeeper(10, print_delay_threshold=None)
    print("3")

    MAX_LAT_ACCEL = 2.5

    while True:
        print("4")
        sm.update(0)

        # Process each message configuration
        for msg_name, msg_config in config.items():
            # Get the struct definition for this message
            struct_name = msg_config["struct_name"]

            # Create a data dictionary
            data = {}

            # Fill in data from the struct fields that are enabled for sending
            for field_name, field_config in msg_config["fields"].items():
                if not field_config["send"]:
                    continue  # Skip fields that aren't marked for sending

                if field_config["path"] == "calculate" and field_name == "max_angle":
                    # Special calculation for max_angle
                    max_curvature = MAX_LAT_ACCEL / max(sm['carState'].vEgo ** 2, 5)
                    data[field_name] = round(math.degrees(VM.get_steer_from_curvature(
                        max_curvature, sm['carState'].vEgo, sm['liveParameters'].roll)), 3)
                elif field_config["path"] == "timestamp":
                    # Add the current timestamp
                    data[field_name] = time.time()
                else:
                    # Get the value from the message using the struct field path
                    try:
                        # Handle nested attributes (e.g., cruiseState.enabled)
                        value = get_nested_attr(sm['carState'], field_config["path"])

                        # Process value based on type
                        data[field_name] = value
                    except AttributeError as e:
                        print(f"Error getting field {field_config['path']}: {e}")
                        data[field_name] = None

            # Serialize data (convert non-serializable types)
            data = serialize_data(data)

            print(f"data: {data}")

            # Send the message
            socket.send_string(f"{msg_name} {json.dumps(data)}")

        rk.keep_time()

if __name__ == "__main__":
    main()
