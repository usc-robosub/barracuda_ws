from importlib.resources import files
import csv
import os
from scipy.interpolate import CubicSpline
from bisect import bisect_right

from .t200_thruster_constants import T200

from . import teensy_constants as TC

# called in teensys.py
# convert force to duty cycle based on luts (derived from pwm freq, res configs)
# if f falls between _f_vals[l_idx] and _f_vals[r_idx], l_idx is returned
def f_to_dc(f: float):
    if f == 0:
        return TC.INIT_DC
    if f <= teensy_f_vals[0]:
        return teensy_dc_vals[0]
    if f >= teensy_f_vals[-1]:
        return teensy_dc_vals[-1]

    # from https://docs.python.org/3/library/bisect.html#bisect-functions
    i = bisect_right(teensy_f_vals, f)

    # take floor of abs(f): if f < 0, "round up", if f > 0, "round down"
    if f > 0:
        # find rightmost index whose val is less than or equal to f
        return teensy_dc_vals[i - 1]
    else: # f < 0
        # find leftmost index whose val is greater than f
        return teensy_dc_vals[i]

## setup the lut lists used by f_to_dc() ##
###########################################

# read in t200 csv file, create a curve to fit the pulse width/force val pairs
CSV_FILE = files("barracuda_thrusters").joinpath(T200.CSV_FILENAME)
NEWTONS_PER_KGF = 9.80665
datasheet_pw_vals = []  # pw_vals: pulse widths
datasheet_f_vals = []  # fs: force values
with CSV_FILE.open() as f:
    reader = csv.DictReader(f)
    for row in reader:
        pw, kgF = int(row["PWM (µs)"]), float(row["Force (Kg f)"])
        datasheet_pw_vals.append(pw)
        datasheet_f_vals.append(kgF * NEWTONS_PER_KGF)

assert len(datasheet_pw_vals) == len(datasheet_f_vals), (
    "datasheet_pw_vals must be same length as datasheet_f_vals"
)
spline = CubicSpline(datasheet_pw_vals, datasheet_f_vals)


# create a list of the possible duty cycle vals that the teensys can use to output
# pwm signal based on the pwm resolution and the min and max pulse widths specified
# in the pulse width/force val csv
def _dc_to_us(dc):
    return dc / (2**TC.PWM_RES) / (TC.PWM_FREQ / 1e6)
teensy_dc_vals = [
    dc for dc in range(2**TC.PWM_RES) if T200.MIN_PW <= _dc_to_us(dc) <= T200.MAX_PW
]


# based on the list of possible duty cycle vals, create a list of the pulse widths
# (in microseconds) that the teensys are able to output for a pwm signal
teensy_pw_vals = [_dc_to_us(dc) for dc in teensy_dc_vals]


# based on the list of possible pulse widths (in microseconds), create a list of
# the possible thruster forces that the teensy pwm outputs can drive
def _pw_to_f(pw):
    return 0 if T200.MIN_ZERO_PW <= pw <= T200.MAX_ZERO_PW else spline(pw)
teensy_f_vals = [_pw_to_f(pw) for pw in teensy_pw_vals]

# run as a standalone script from outer barracuda_thrusters dir: python -m barracuda_thrusters.force_to_pwm_dc
if __name__ == "__main__":
    for i in range(len(teensy_dc_vals)):
      print(i, teensy_dc_vals[i], teensy_pw_vals[i], teensy_f_vals[i])
    
