#!/bin/bash
gnome-terminal -- roslaunch autocamera test_hardware.launch &&
gnome-terminal -- rosrun dvrk_autocamera autocamera_control_node.py &&
gnome-terminal -- rosrun dvrk_assistant_bridge bridge &&
source ~/anaconda3/etc/profile.d/conda.sh
gnome-terminal -- conda init bash && conda activate vosk && python test_mic_vad.py -l && python test_mic_vad.py -d 12