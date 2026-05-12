# barracuda_thrusters
this package contains software that interfaces with the microcontrollers on the vehicle's thruster boards.

## ros node
* teensy comms node subscribes to the cmd_thrust topic (which the thruster manager node publishes to), the topic's msg type is [JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html) and its "effort" field is an array of the desired thruster force outputs, indexed by urdf thruster numbering
* converts desired force outputs to pwm duty cycle values based on [t200 thruster force-to-pwm width chart](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/) and the pwm configuration of the teensys specified in teensy_constants.py (see [teensy pwm docs](https://www.pjrc.com/teensy/td_pulse.html)) - conversion code in force_to_pwm_dc.py
* sends pwm duty cycle vals 0-3 (converted from desired thruster force outputs 0-3) to the teensy on the port-side thruster board, and duty cycle vals 4-7 to the teensy on the starboard-side thruster board, using a Teensys object's set_pwm_outputs function (Teensys class defined in teensy.py)
* the Teensys class also sets up callbacks triggered on level changes of the jetson's killswitch/enable gpio pin, which enable/disable regular pwm output on the teensys

## teensy code
* teensy code is located in barracuda_thrusters/teensy_pwm_output/ dir; both the python code used by the ros node and the teensy code get their teensy-specific values from the same file (teensy_constants.py), so that these values only need to be updated/modified in one place
* receives pwm duty cycle vals, enbable signal over i2c
* when enable signal changes, reinits thrusters
* when regular pwm output is disabled, ignores new writes to duty cycle register (pwm output will stay at init pulse width), indicator led blinks slower
* when regular pwm output is enabled, writes new pwm output upon writes to duty cycle register, indicator led stays on without blinking when desired force output for all thrusters driven by teensy is zero, blinks faster otherwise

### setting up arduino environment for compiling & flashing teensy code
* [install arduino-cli](https://docs.arduino.cc/arduino-cli/installation/)
* run ```./arduino-cli_setup.sh``` to install teensy4 core, teensy4_i2c library
* edit teensy_pwm_output.ino with Arduino IDE or your text editor of choice
* run ```./compile <PORT|STARBOARD> [-u|--upload /usb/port/used/by/teensy]``` to compile the sketch and optionally flash the teensy


## file summaries
```teensy_comms_node.py```: ros node executable launched in launch/barracuda_thrusters.launch.py
```teensys.py```: defines the Teensys class
```jetson_gpio_utils.py```: i2c, jetson gpio utility functions
```teensy_constants.py```: teensy i2c, pwm config vals
```t200_thruster_constants.py```: t200 thruster pwm-related constants
### teensy_pwm_output/
```teensy_pwm_output.ino```: main teensy code
```teensy_config.h```: teensy i2c, pwm values pulled from teensy_constants.py
```update_teensy_config_header.py```: updates teensy_config.h with current vals from teensy_constants.py
```arduino-cli_setup.sh```: sets up arduino environment, installs teensy4_i2c library dependency
```compile.sh```: runs update_teensy_config_header.py, compiles teensy_pwm_output.ino, optionally flashes teensy

