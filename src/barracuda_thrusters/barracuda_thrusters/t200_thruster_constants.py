# t200 thruster constants
class T200:
    # csv file with pwm width/force chart (18V sheet from https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/)
    CSV_FILENAME = "t200_18v_data.csv"

    # t200 thruster pulse width constants in microseconds
    INIT_PW = 1500
    MIN_PW = 1100
    MAX_PW = 1900
    MIN_ZERO_PW = 1472
    MAX_ZERO_PW = 1528

    NTHRUSTERS = 8
