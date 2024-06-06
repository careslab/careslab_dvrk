import queue
import sys
import argparse

import numpy as np
import sounddevice as sd
import torch
from transformers import WhisperProcessor, WhisperForConditionalGeneration

processor = WhisperProcessor.from_pretrained("openai/whisper-tiny")
model = WhisperForConditionalGeneration.from_pretrained("openai/whisper-tiny").to('cuda')
# forced_decoder_ids = processor.get_decoder_prompt_ids(language="english", task="transcribe")
# print(forced_decoder_ids)
prompt_ids = processor.get_prompt_ids("The next sentence I am going to say is a command for a davinci surgical robot.")
q = queue.Queue()

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text


def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(indata)


parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
    '-l', '--list-devices', action='store_true',
    help='show list of audio devices and exit')
args, remaining = parser.parse_known_args()
if args.list_devices:
    print(sd.query_devices())
    parser.exit(0)

parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
    parents=[parser])
parser.add_argument(
    '-f', '--filename', type=str, metavar='FILENAME',
    help='audio file to store recording to')
parser.add_argument(
    '-d', '--device', type=int_or_str,
    help='input device (numeric ID or substring)', nargs='?')
parser.add_argument(
    '-r', '--samplerate', type=int, help='sampling rate', nargs='?', default=16000)
args = parser.parse_args(remaining)
print(args.samplerate)
with torch.no_grad():
    with sd.InputStream(samplerate=args.samplerate, blocksize=4000, device=args.device, dtype='float32',
                        channels=1, callback=callback):
        print('#' * 80)
        print('Press Ctrl+C to stop the recording')
        print('#' * 80)
        while True:
            data = np.squeeze(q.get(), -1)  # get into vector/array form
            input_features = processor(data, sampling_rate=args.samplerate, return_tensors="pt",
                        pad_to_multiple_of=128, do_normalize=True).input_features
            input_features = input_features.to('cuda')
            predicted_ids = model.generate(input_features,prompt_ids=prompt_ids)
            transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)
            print(transcription)