import argparse
import os
import queue
import sounddevice as sd
import vosk
import json
import sys
import threading
from playsound import playsound
# from timeit import default_timer as timer
from transformers import pipeline
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16
# from timeit import default_timer as timer

threading.Thread(target=lambda: rospy.init_node('dvrk_voice', disable_signals=True, anonymous=True)).start()

run_pub = rospy.Publisher('/assistant/autocamera/run', Bool, queue_size=1, latch=True)
track_pub = rospy.Publisher('/assistant/autocamera/track', String, queue_size=1, latch=True)
keep_pub = rospy.Publisher('/assistant/autocamera/keep', String, queue_size=1, latch=True)
findtools_pub = rospy.Publisher('/assistant/autocamera/find_tools', Empty, queue_size=1, latch=True)
innerZoom_pub = rospy.Publisher('/assistant/autocamera/inner_zoom_value', Float32, queue_size=1, latch=True)
outerZoom_pub = rospy.Publisher('/assistant/autocamera/outer_zoom_value', Float32, queue_size=1, latch=True)
saveEcm_pub = rospy.Publisher('/assistant/save_ecm_position', Int16, queue_size=1)
gotoEcm_pub = rospy.Publisher('/assistant/goto_ecm_position', Int16, queue_size=1)
voiceCommand_pub = rospy.Publisher('/voice_command', String, queue_size=1)
picture_pub = rospy.Publisher('/assistant/take_picture', Bool, queue_size=1)
video_pub = rospy.Publisher('/assistant/take_video', Bool, queue_size=1)

q = queue.Queue()


# classifier = pipeline("zero-shot-classification", model="cross-encoder/nli-MiniLM2-L6-H768",device=0)
classifier = pipeline("zero-shot-classification", model="MoritzLaurer/DeBERTa-v3-xsmall-mnli-fever-anli-ling-binary",device=0)
choices = ['davinci track right',  'davinci track left', 'davinci track middle', 
           'davinci start',  'davinci stop', 'davinci keep left', 'davinci keep right', 'davinci my find tools', 
           'davinci take a picture', 'davinci start video', 'davinci stop video', 'Something not valid'] # candidate labels


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
    q.put(bytes(indata))

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
    '-m', '--model', type=str, metavar='MODEL_PATH',
    help='Path to the model')
parser.add_argument(
    '-d', '--device', type=int_or_str,
    help='input device (numeric ID or substring)')
parser.add_argument(
    '-r', '--samplerate', type=int, help='sampling rate')
args = parser.parse_args(remaining)

try:
    if args.model is None:
        args.model = "model_o"
    if not os.path.exists(args.model):
        print ("Please download a model for your language from https://alphacephei.com/vosk/models")
        print ("and unpack as 'model' in the current folder.")
        parser.exit(0)                       #We then ask GPT to figure out what the user wants to be done.
                        #the function returns an index to the list of items that are in choices.
                        # print(output)
    if args.samplerate is None:
        device_info = sd.query_devices(args.device, 'input')
        # soundfile expects an int, sounddevice provides a float:
        args.samplerate = int(device_info['default_samplerate'])

    model = vosk.Model(args.model)
    dump_time = open("./timings_query_c-e.txt","w")
    if args.filename:
        dump_fn = open(args.filename, "wb")
    else:
        dump_fn = None

 
    with sd.RawInputStream(samplerate=args.samplerate, blocksize = 4000, device=args.device, dtype='int16',
                            channels=1, callback=callback):
            print('#' * 80)
            print('Press Ctrl+C to stop the recording')
            print('#' * 80)

            rec = vosk.KaldiRecognizer(model, args.samplerate)
            prevprompt =""
            prompt = ""
            cmd =""

            while True:
                data = q.get()
                if rec.AcceptWaveform(data):
                    #publish to run topic 
                    res = json.loads(rec.Result())
                    prompt = res['text']


                else:
                    res = json.loads(rec.PartialResult())
                    prevprompt = prompt
                    prompt = res['partial']
                    
                    if ((res['partial'] == "") and (prompt == "") and (prevprompt != "")) :
                        # start = timer()
                        print (prevprompt)
                        output = classifier(prevprompt, choices)
                        cmd = output['labels'][0]
                        prevpromt ="" #wipe out the previous prompt
                        
                    
                    #execute the command
                    if(cmd == "davinci start auto camera" or cmd == "davinci start"):
                        print("Running autocamera")
                        rospy.Publisher('/assistant/clutch_and_move/run', Bool, latch=True, queue_size=1).publish(Bool(False))
                        rospy.Publisher('/assistant/joystick/run', Bool, latch=True, queue_size=1).publish(Bool(False))
                        rospy.Publisher('/assistant/oculus/run', Bool, latch=True, queue_size=1).publish(Bool(False))
                        rospy.Publisher('/assistant/clutchless/run', Bool, latch=True, queue_size=1).publish(Bool(False))
                        run_pub.publish(True)
                        voiceCommand_pub.publish("Da Vinci Start")
                        playsound('sound95.wav')
                        rec.Reset()

                    elif(cmd == "davinci stop auto camera" or cmd == "davinci stop"):
                        print("Stopping autocamera")
                        run_pub.publish(False)
                        voiceCommand_pub.publish("Da Vinci Stop")
                        playsound('sound95.wav')
                        # end = timer()
                        # dump_time.write(f"stop {end-start}\n")
                        rec.Reset()

                    #publish to track topic 
                    elif(cmd == "davinci track right"):
                        print("Tracking right")
                        track_pub.publish("right")
                        voiceCommand_pub.publish("Da Vinci track right")
                        playsound('sound95.wav')
                        # end = timer()
                        # dump_time.write(f"track right {end-start}\n")
                        rec.Reset()

                    elif(cmd == "davinci track left"):
                        print("Tracking left")
                        track_pub.publish("left")
                        voiceCommand_pub.publish("Da Vinci track left")
                        # end = timer()
                        # dump_time.write(f"track left {end-start}\n")
                        playsound('sound95.wav')
                        rec.Reset()

                    elif(cmd == "davinci track middle"or \
                         cmd == "davinci track off"):
                        print("Tracking middle")
                        track_pub.publish("middle")
                        voiceCommand_pub.publish("Da Vinci track middle")
                        # end = timer()
                        # dump_time.write(f"track middle {end-start}\n")
                        playsound('sound95.wav')
                        rec.Reset()

                    #publish to keep topic 
                    elif(cmd == "davinci keep right"):
                        print("Keeping right")
                        keep_pub.publish("right")
                        voiceCommand_pub.publish("Da Vinci keep right")
                        # end = timer()
                        # dump_time.write(f"keep right {end-start}\n")
                        playsound('sound95.wav')
                        rec.Reset()

                    elif(cmd == "davinci keep left"):
                        print("Keeping left")
                        keep_pub.publish("left")
                        voiceCommand_pub.publish("Da Vinci keep left")
                        # end = timer()
                        # dump_time.write(f"keep left {end-start}\n")
                        playsound('sound95.wav')

                        rec.Reset()

                    elif(cmd == "davinci keep middle" or \
                         cmd == "davinci keep off"):
                        print("Keeping middle")
                        keep_pub.publish("middle")
                        voiceCommand_pub.publish("Da Vinci keep middle")
                        # end = timer()
                        # dump_time.write(f"keep middle {end-start}\n")
                        playsound('sound95.wav')
                        rec.Reset()

                    #publish to find tools topic
                    elif(cmd == "davinci find tools" or \
                         cmd == "davinci find my tools"):
                        print("Finding tools")
                        findtools_pub.publish()
                        voiceCommand_pub.publish("Da Vinci find my tools")
                        # end = timer()
                        # dump_time.write(f"find tools {end-start}\n")
                        playsound('sound95.wav')
                        rec.Reset()
                        
                    elif cmd == "davinci take a picture":
                        print("Taking picture")
                        picture_pub.publish(True)
                        voiceCommand_pub.publish("Da Vinci take picture")
                        # end = timer()
                        # dump_time.write(f"take picture {end-start}\n")
                        playsound('sound95.wav')
                        picture_pub.publish(False)
                        rec.Reset()

                    elif cmd == "davinci start video":
                        print("Starting video")
                        video_pub.publish(True)
                        voiceCommand_pub.publish("Da Vinci begin recording")
                        # end = timer()
                        # dump_time.write(f"start video {end-start}\n")
                        playsound('sound95.wav')
                        rec.Reset()

                    elif cmd == "davinci stop video":
                        print("Stopping video")
                        video_pub.publish(False)
                        voiceCommand_pub.publish("Da Vinci end recording")
                        # end = timer()
                        # dump_time.write(f"stop video {end-start}\n")
                        playsound('sound95.wav')
                        rec.Reset()
                  
                    elif cmd == "Something not valid":
                        print("Not valid")
                        #playsound('repeat.wav')
                        #if the system responds with I don't know...send it a long command again
                        
                        rec.Reset()

                    else:

                        pass

                    #print(rec.PartialResult())
                #reset the command.
                cmd =""
                

                if dump_fn is not None:
                    dump_fn.write(data)
                    dump_fn.close()
                    dump_time.close()

                    
            
except KeyboardInterrupt:
    print('\nDone')
    parser.exit(0)
except Exception as e:
    parser.exit(type(e).__name__ + ': ' + str(e))

