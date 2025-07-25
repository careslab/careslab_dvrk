#!/bin/bash

gnome-terminal -- bash -c "source transformers-env/bin/activate && python whisper_roberta.py -l && python whisper_roberta.py -d 10; exec bash"