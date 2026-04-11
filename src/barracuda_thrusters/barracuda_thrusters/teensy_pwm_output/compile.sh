#!/bin/bash

if [[ $# -ne 1 && $# -ne 3 ]]; then
  echo "Usage: $0 <PORT|STARBOARD> [-u|--upload /usb/port]" >&2
  exit 1
fi

if [[ "$1" != "PORT" && "$1" != "STARBOARD" ]]; then
  echo "Error: first argument must be PORT or STARBOARD, got '$1'" >&2
  exit 1
fi

if [[ $# -eq 3 && "$2" != "-u" && "$2" != "--upload" ]]; then
  echo "Error: second argument must be -u or --upload, got '$2'" >&2
  exit 1
fi

# updates teensy_config.h
python3 update_teensy_config_header.py

# see https://arduino.github.io/arduino-cli/dev/commands/arduino-cli_compile/

# default: "build.flags.defs=-D__IMXRT1062__ -DTEENSYDUINO=160"
# see defaults with arduino-cli compile -b teensy:avr:teensy40 --show-properties
# add -DPORT or -DSTARBOARD so that appropriate I2C address gets assigned in .ino file

# we shorten -u|--upload -p /usb/device used by arduino-cli to -u|--upload /usb/device
# see -p, -u options in link above

arduino-cli compile -b teensy:avr:teensy40 \
--build-property "build.flags.defs=-D__IMXRT1062__ -DTEENSYDUINO=160 -D$1" \
${2:+"$2" "-p" "$3"}
