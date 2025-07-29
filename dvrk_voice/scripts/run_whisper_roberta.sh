#!/bin/bash

gnome-terminal -- bash -c "cd /home/careslab/careslab_dvrk/dvrk_voice/scripts && python whisper_roberta.py -l && python whisper_roberta.py -d 10; exec bash"
