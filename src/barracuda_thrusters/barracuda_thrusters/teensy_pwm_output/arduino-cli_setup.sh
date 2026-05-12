#!/bin/bash

# installs teensy core: https://www.pjrc.com/teensy/td_download.html
arduino-cli core install teensy:avr --additional-urls "https://www.pjrc.com/teensy/package_teensy_index.json"

# allows --git-url option to be used with "lib install" command
arduino-cli config set library.enable_unsafe_install true

# install teensy4_i2c library: https://github.com/Richard-Gemmell/teensy4_i2c/tree/master
arduino-cli lib install --git-url "https://github.com/Richard-Gemmell/teensy4_i2c.git"
