#!/bin/bash

gnome-terminal -- bash -c "cd ~/catkin_ws/src/careslab_dvrk/dvrk_voice/scripts && python whisper_roberta.py -l && python whisper_roberta.py -d 10; exec bash"
