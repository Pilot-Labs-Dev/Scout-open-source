#!/bin/sh

ldconfig

string=$(awk '{print $2}' /var/roller_eye/config/soundEffect | awk -F "," '{print $1}' | awk 'NR==2')
if [ $string = '1' ]; then
sudo aplay /var/roller_eye/devAudio/power_on_down/power_on.wav &
fi
