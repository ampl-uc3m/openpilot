from collections import deque
from cereal import car, log
import cereal.messaging as messaging
from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import MAX_CTRL_SPEED
from opendbc.car.volkswagen.values import CarControllerParams as VWCarControllerParams
from opendbc.car.hyundai.interface import ENABLE_BUTTONS as HYUNDAI_ENABLE_BUTTONS
from opendbc.car.hyundai.carstate import PREV_BUTTON_SAMPLES as HYUNDAI_PREV_BUTTON_SAMPLES

from openpilot.selfdrive.selfdrived.events import Events

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
EventName = log.OnroadEvent.EventName
NetworkLocation = structs.CarParams.NetworkLocation


# TODO: the goal is to abstract this file into the CarState struct and make events generic
class MockCarState:
  def __init__(self):
    self.sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal'])

  def update(self, CS: car.CarState):
    self.sm.update(0)
    gps_sock = 'gpsLocationExternal' if self.sm.recv_frame['gpsLocationExternal'] > 1 else 'gpsLocation'

    CS.vEgo = self.sm[gps_sock].speed
    CS.vEgoRaw = self.sm[gps_sock].speed

    return CS


class CarSpecificEvents:
  def __init__(self, CP: structs.CarParams): # type: ignore
    self.CP = CP

    self.steering_unpressed = 0
    self.low_speed_alert = False
    self.no_steer_warning = False
    self.silent_steer_warning = True

    self.cruise_buttons: deque = deque([], maxlen=HYUNDAI_PREV_BUTTON_SAMPLES)

  def update(self, CS: car.CarState, CS_prev: car.CarState, CC: car.CarControl):
    if self.CP.brand in ('body', 'mock'):
      events = Events()

    elif self.CP.brand == 'ford':
      events = self.create_common_events(CS, CS_prev, extra_gears=[GearShifter.manumatic])

    elif self.CP.brand == 'nissan':
      events = self.create_common_events(CS, CS_prev, extra_gears=[GearShifter.brake])

    elif self.CP.brand == 'chrysler':
      events = self.create_common_events(CS, CS_prev, extra_gears=[GearShifter.low])

      # Low speed steer alert hysteresis logic
      if self.CP.minSteerSpeed > 0. and CS.vEgo < (self.CP.minSteerSpeed + 0.5):
        self.low_speed_alert = True
      elif CS.vEgo > (self.CP.minSteerSpeed + 1.):
        self.low_speed_alert = False
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

    elif self.CP.brand == 'honda':
      events = self.create_common_events(CS, CS_prev, pcm_enable=False)

      if self.CP.pcmCruise and CS.vEgo < self.CP.minEnableSpeed:
        events.add(EventName.belowEngageSpeed)

      if self.CP.pcmCruise:
        # we engage when pcm is active (rising edge)
        if CS.cruiseState.enabled and not CS_prev.cruiseState.enabled:
          events.add(EventName.pcmEnable)
        elif not CS.cruiseState.enabled and (CC.actuators.accel >= 0. or not self.CP.openpilotLongitudinalControl):
          # it can happen that car cruise disables while comma system is enabled: need to
          # keep braking if needed or if the speed is very low
          if CS.vEgo < self.CP.minEnableSpeed + 2.:
            # non loud alert if cruise disables below 25mph as expected (+ a little margin)
            events.add(EventName.speedTooLow)
          else:
            events.add(EventName.cruiseDisabled)
      if self.CP.minEnableSpeed > 0 and CS.vEgo < 0.001:
        events.add(EventName.manualRestart)

    elif self.CP.brand == 'toyota':
      events = self.create_common_events(CS, CS_prev)

      if self.CP.openpilotLongitudinalControl:
        if CS.cruiseState.standstill and not CS.brakePressed:
          events.add(EventName.resumeRequired)
        if CS.vEgo < self.CP.minEnableSpeed:
          events.add(EventName.belowEngageSpeed)
          if CC.actuators.accel > 0.3:
            # some margin on the actuator to not false trigger cancellation while stopping
            events.add(EventName.speedTooLow)
          if CS.vEgo < 0.001:
            # while in standstill, send a user alert
            events.add(EventName.manualRestart)

    elif self.CP.brand == 'gm':
      events = self.create_common_events(CS, CS_prev, extra_gears=[GearShifter.sport, GearShifter.low,
                                                                   GearShifter.eco, GearShifter.manumatic],
                                         pcm_enable=self.CP.pcmCruise)

      # Enabling at a standstill with brake is allowed
      # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
      if CS.vEgo < self.CP.minEnableSpeed and not (CS.standstill and CS.brake >= 20 and
                                                   self.CP.networkLocation == NetworkLocation.fwdCamera):
        events.add(EventName.belowEngageSpeed)
      if CS.cruiseState.standstill:
        events.add(EventName.resumeRequired)
      if CS.vEgo < self.CP.minSteerSpeed:
        events.add(EventName.belowSteerSpeed)

    elif self.CP.brand == 'volkswagen':
      events = self.create_common_events(CS, CS_prev, extra_gears=[GearShifter.eco, GearShifter.sport, GearShifter.manumatic],
                                         pcm_enable=self.CP.pcmCruise)

      # Low speed steer alert hysteresis logic
      if (self.CP.minSteerSpeed - 1e-3) > VWCarControllerParams.DEFAULT_MIN_STEER_SPEED and CS.vEgo < (self.CP.minSteerSpeed + 1.):
        self.low_speed_alert = True
      elif CS.vEgo > (self.CP.minSteerSpeed + 2.):
        self.low_speed_alert = False
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

      if self.CP.openpilotLongitudinalControl:
        if CS.vEgo < self.CP.minEnableSpeed + 0.5:
          events.add(EventName.belowEngageSpeed)
        if CC.enabled and CS.vEgo < self.CP.minEnableSpeed:
          events.add(EventName.speedTooLow)

      # TODO: this needs to be implemented generically in carState struct
      # if CC.eps_timer_soft_disable_alert:  # type: ignore[attr-defined]
      #   events.add(EventName.steerTimeLimit)

    elif self.CP.brand == 'hyundai':
      # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
      # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
      # Main button also can trigger an engagement on these cars
      self.cruise_buttons.append(any(ev.type in HYUNDAI_ENABLE_BUTTONS for ev in CS.buttonEvents))
      events = self.create_common_events(CS, CS_prev, extra_gears=(GearShifter.sport, GearShifter.manumatic),
                                         pcm_enable=self.CP.pcmCruise, allow_enable=any(self.cruise_buttons), allow_button_cancel=False)

      # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
      if CS.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
        self.low_speed_alert = True
        #print("Hyundai: Low speed alert ACTIVATED") # Added print
      if CS.vEgo > (self.CP.minSteerSpeed + 4.):
        self.low_speed_alert = False
        #print("Hyundai: Low speed alert DEACTIVATED") # Added print
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)
        #print("Hyundai: belowSteerSpeed event added") # Added print

    else:
      events = self.create_common_events(CS, CS_prev)

    return events

  def create_common_events(self, CS: structs.CarState, CS_prev: car.CarState, extra_gears=None, pcm_enable=True,
                           allow_enable=True, allow_button_cancel=True):
    events = Events()

    if CS.doorOpen:
      events.add(EventName.doorOpen)
      print("doorOpen Event Added") # Added print
    if CS.seatbeltUnlatched:
      events.add(EventName.seatbeltNotLatched)
      print("seatbeltNotLatched Event Added") # Added print
    if CS.gearShifter != GearShifter.drive and (extra_gears is None or
       CS.gearShifter not in extra_gears):
      events.add(EventName.wrongGear)
      #print("wrongGear Event Added") # Added print
    if CS.gearShifter == GearShifter.reverse:
      events.add(EventName.reverseGear)
      #print("reverseGear Event Added") # Added print
    if not CS.cruiseState.available:
      events.add(EventName.wrongCarMode)
      #print("wrongCarMode Event Added") # Added print
    if CS.espDisabled:
      events.add(EventName.espDisabled)
      print("espDisabled Event Added") # Added print
    if CS.espActive:
      events.add(EventName.espActive)
      print("ESP Active Event Added") # Added print
    if CS.stockFcw:
      events.add(EventName.stockFcw)
      print("Stock FCW Event Added") # Added print
    if CS.stockAeb:
      events.add(EventName.stockAeb)
      print("Stock AEB Event Added") # Added print
    if CS.vEgo > MAX_CTRL_SPEED:
      events.add(EventName.speedTooHigh)
      print("Speed Too High Event Added") # Added print
    if CS.cruiseState.nonAdaptive:
      events.add(EventName.wrongCruiseMode)
      print("Wrong Cruise Mode Event Added") # Added print
    if CS.brakeHoldActive and self.CP.openpilotLongitudinalControl:
      events.add(EventName.brakeHold)
      print("Brake Hold Event Added") # Added print
    if CS.parkingBrake:
      events.add(EventName.parkBrake)
      print("Park Brake Event Added") # Added print
    if CS.accFaulted:
      events.add(EventName.accFaulted)
      print("ACC Faulted Event Added") # Added print
    if CS.steeringPressed:
      events.add(EventName.steerOverride)
      #print("Steer Override Event Added") # Added print
    if CS.brakePressed and CS.standstill:
      events.add(EventName.preEnableStandstill)
      #print("Pre Enable Standstill Event Added") # Added print
    if CS.gasPressed:
      events.add(EventName.gasPressedOverride)
      #print("Gas Pressed Override Event Added") # Added print
    if CS.vehicleSensorsInvalid:
      events.add(EventName.vehicleSensorsInvalid)
      print("Vehicle Sensors Invalid Event Added") # Added print
    if CS.invalidLkasSetting:
      events.add(EventName.invalidLkasSetting)
      print("Invalid LKAS Setting Event Added") # Added print
    if CS.lowSpeedAlert:
      events.add(EventName.belowSteerSpeed)
      print("Below Steer Speed Event Added") # Added print
    if CS.buttonEnable:
      events.add(EventName.buttonEnable)
      print("Button Enable Event Added") # Added print

    # Handle cancel button presses
    for b in CS.buttonEvents:
      # Disable on rising and falling edge of cancel for both stock and OP long
      # TODO: only check the cancel button with openpilot longitudinal on all brands to match panda safety
      if b.type == ButtonType.cancel and (allow_button_cancel or not self.CP.pcmCruise):
        events.add(EventName.buttonCancel)
        print("Cancel Button pressed!") # Added print

    # Handle permanent and temporary steering faults
    self.steering_unpressed = 0 if CS.steeringPressed else self.steering_unpressed + 1
    if CS.steerFaultTemporary:
      if CS.steeringPressed and (not CS_prev.steerFaultTemporary or self.no_steer_warning):
        self.no_steer_warning = True
      else:
        self.no_steer_warning = False

        # if the user overrode recently, show a less harsh alert
        if self.silent_steer_warning or CS.standstill or self.steering_unpressed < int(1.5 / DT_CTRL):
          self.silent_steer_warning = True
          events.add(EventName.steerTempUnavailableSilent)
        else:
          events.add(EventName.steerTempUnavailable)
    else:
      self.no_steer_warning = False
      self.silent_steer_warning = False
    if CS.steerFaultPermanent:
      events.add(EventName.steerUnavailable)

    # we engage when pcm is active (rising edge)
    # enabling can optionally be blocked by the car interface
    if pcm_enable:
      if CS.cruiseState.enabled and not CS_prev.cruiseState.enabled and allow_enable:
        events.add(EventName.pcmEnable)
        #print("pcmEnable Event Added!")
      elif not CS.cruiseState.enabled:
        events.add(EventName.pcmDisable)
        #print("pcmDisable Event Added!")

    return events
