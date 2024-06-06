#!/bin/bash
source ~/anaconda3/etc/profile.d/conda.sh
gnome-terminal -- conda init bash && conda activate vosk && python3 test_mic_vad.py -l && python3 test_mic_vad.py -d 3
