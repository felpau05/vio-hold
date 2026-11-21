"""The event GNSS rule, as FC parameter values.

Source: the event Stage 2 rules (Revision 2.0), sections 01 and 04.
1. From the moment the engage switch goes on until it goes off, the solution must not use GNSS
   ("GPS-aided state" included). Use of GNSS during a hold voids that flight.
2. GPS stays connected: the judges measure the drift from the GPS in the FC log.

The rule is fixed in this file. A profile or a settings.json edit cannot change it:
- settings.load refuses a params_ardupilot or params_px4 value that breaks it, so
  'check --set-params' cannot write such a value to the FC.
- apb_main run refuses to start when the FC has such a value, and check prints one line per parameter.
A parameter that does not answer is a FAIL: then the program cannot show that the FC uses no GPS.
"""

# ArduPilot EKF3 source values that are GNSS (AP_NavEKF_Source): POSXY, VELXY, POSZ, VELZ 3 = GPS.
# YAW 2 = GPS, 3 = GPS with compass fallback, 8 = GSF (the GSF uses GPS velocity).
# All three source sets are checked: an RC switch (RCx_OPTION 90) can select any set in flight.
AP_FORBIDDEN = {f"EK3_SRC{n}_{k}": bad for n in (1, 2, 3)
                for k, bad in (("POSXY", (3,)), ("VELXY", (3,)), ("POSZ", (3,)), ("VELZ", (3,)), ("YAW", (2, 3, 8)))}
# These must be 0.
# AHRS_GPS_USE: 1 or 2 lets the DCM fallback (used when the EKF is not healthy) use GPS.
# EK3_GSF_RUN_MASK, EK3_GSF_USE_MASK: the EKF-GSF yaw estimator uses GPS velocity and can reset the EKF yaw.
# All three names are in the event firmware (ArduCopter 4.7.2 fb773567, checked in its binary).
AP_ZERO = ("AHRS_GPS_USE", "EK3_GSF_RUN_MASK", "EK3_GSF_USE_MASK")
# GPS on: GPS1_TYPE (GPS_TYPE before ArduPilot 4.6) 0 means no GPS, so no GPS in the FC log.
AP_GPS_TYPE = ("GPS1_TYPE", "GPS_TYPE")
# LOG_BITMASK bits that the judges need (ArduCopter defines.h): GPS for the drift, RCIN for the engage switch.
AP_LOG_BITS = {"GPS": 1 << 2, "RCIN": 1 << 6}
# PX4 (no PX4 build exists for the event board, Stage 2 section 01): EKF2_GPS_CTRL 0 = the EKF fuses no GPS.
PX4_ZERO = ("EKF2_GPS_CTRL",)

# settings.py params_ardupilot and params_px4 hold a good value for these, so 'check --set-params' writes it.
# For the other names (EK3_SRC2_*, EK3_SRC3_*, LOG_BITMASK), use the ground station.
SET_PARAMS_FIXES = ("EK3_SRC1_", "AHRS_GPS_USE", "EK3_GSF_", "GPS1_TYPE", "GPS_TYPE", "EKF2_GPS_CTRL")


def fix_text(name):
    how = ("Run 'apb_main.py check --set-params'" if name.startswith(SET_PARAMS_FIXES) else
           "Set it in the ground station (0 is allowed)" if name.startswith("EK3_SRC") else
           "Set it in the ground station")
    return how + ", then reboot the FC."


def value_problem(firmware, name, v):
    """The reason why value v of parameter name breaks the GNSS rule, or None."""
    if firmware == "ardupilot":
        if name in AP_FORBIDDEN and round(v) in AP_FORBIDDEN[name]:
            return f"{name} {v:g} makes the EKF use GPS"
        if name in AP_ZERO and v != 0:
            return f"{name} {v:g} lets ArduPilot use GPS (need 0)"
        if name in AP_GPS_TYPE and v == 0:
            return f"{name} 0 turns GPS off, and the judges need GPS in the FC log (need 1)"
        if name == "LOG_BITMASK":
            missing = [k for k, bit in AP_LOG_BITS.items() if not int(v) & bit]
            if missing:
                return f"LOG_BITMASK {int(v)} does not log {' and '.join(missing)}, and the judges need it"
    elif firmware == "px4" and name in PX4_ZERO and v != 0:
        return f"{name} {v:g} makes the EKF use GPS (need 0)"
    return None


def check(firmware, get):
    """[(ok, line)] for an FC. get(name) returns the value, or None when the FC does not answer.
    Each line starts with 'PASS ' or 'FAIL ', like fc_link.check_params."""
    if firmware == "ardupilot":
        names = list(AP_FORBIDDEN) + list(AP_ZERO) + ["LOG_BITMASK"]
    elif firmware == "px4":
        names = list(PX4_ZERO)
    else:
        return []
    out = []
    for name in names:
        out.append(_line(firmware, name, get(name)))
    if firmware == "ardupilot":
        name, v = AP_GPS_TYPE[0], get(AP_GPS_TYPE[0])
        if v is None:
            name, v = AP_GPS_TYPE[1], get(AP_GPS_TYPE[1])
        out.append(_line(firmware, name, v))
    return out


def _line(firmware, name, v):
    if v is None:
        return False, (f"FAIL GNSS rule: {name} does not answer, so the program cannot show that the FC "
                       f"uses no GPS. Check the parameter name for this firmware version.")
    why = value_problem(firmware, name, v)
    return (True, f"PASS GNSS rule: {name} = {v:g}") if why is None else (False, f"FAIL GNSS rule: {why}. {fix_text(name)}")
