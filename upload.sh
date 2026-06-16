#!/bin/bash

arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old nano
arduino-cli upload -p /dev/ttyUSB* --fqbn arduino:avr:nano:cpu=atmega328old -v nano
