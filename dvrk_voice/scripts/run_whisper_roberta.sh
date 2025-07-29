#!/bin/bash

source ./data/transformers-env/bin/activate
python whisper_roberta.py -l
python whisper_roberta.py -d 10
