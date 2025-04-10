#!/usr/bin/env python3
import math
import zmq
import time
import json

from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, DT_CTRL
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from cereal import messaging, car

def main():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5560")  # puerto para publicar el max_angle

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

        max_curvature = MAX_LAT_ACCEL / max(sm['carState'].vEgo ** 2, 5)
        max_angle = math.degrees(VM.get_steer_from_curvature(max_curvature, sm['carState'].vEgo, sm['liveParameters'].roll))

        cruise_control_enabled = sm['carState'].cruiseState.enabled

        print(f"max angle: {max_angle:.2f}")

        print("cruise enbled", cruise_control_enabled)


        # Crear el mensaje con todos los valores
        data = {
            "max_angle": round(max_angle, 3),
            "cruise_control_enabled": int(cruise_control_enabled),
            "timestamp": time.time()
        }

        print ("data", data)

        socket.send_string("car_state " + json.dumps(data))

        rk.keep_time()

if __name__ == "__main__":
    main()
