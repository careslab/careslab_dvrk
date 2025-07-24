import os
import tempfile 
import sounddevice as sd
import soundfile as sf
import whisper 
import json
import sys
import threading
from playsound import playsound

import openai

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16

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


#Chat GPT routines and limited choices. 

#define choice for chatgpt
choices = {'TR': 'davinci track right', 'TL': 'davinci track left', 'TM': 'davinci track middle', 
           'ST': 'davinci start', 'SX': 'davinci stop', 'KL': 'davinci keep left', 'KR': 'davinci keep right', 'FT': 'davinci find tools', 
           'TP': 'davinci take picture', 'SV': 'davinci start video', 'XV': 'davinci stop video', 'NV': 'Something not valid'}

#we limit the output of chattpt to only these commands. 
listofpossiblecommands = "TR TL TM ST SX KL KR FT TP SV XV NV"

#get the key
openai.api_key = os.getenv('OPENAI_API_KEY')

def AskGPT (prompt):
####################################################################
# This function takes a command like 'plese track the left tool' and 
# provides chatgpt some examples on how to answer the command.  Note
# that the return value is only one of the choices in the choices array
# above.   These choices should all have actions in the calling program.
#####################################################################
    completions = openai.ChatCompletion.create(
        model="gpt-4o",
        temperature = 0.7,
        messages=[
            #realtime training data... we can limit this to 3-4 times and then simplify the command to just the last part. 
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": "Given this phrase: 'Track the right tool' return the letters correspoing to the right answer: 'TR': 'track or follow the right tool', 'TL': 'track or follow the left' \
                'TM': 'track or follow the middle of the tools', 'ST': 'start or start moving camera', 'SX': 'stop or stop everything', 'KL': 'keep the left tool in view', 'KR': 'keep the right tool in view', 'FT': 'davinci find my tools', 'TP': 'take picture',\
                'SV': 'indication to start a video recording', 'XV': 'indications to stop the video recording', 'NV': 'something not valid or not understood"
            },
            {"role": "assistant", "content": "TR"},

            {"role": "user", "content": "Start playing the video"},
            {"role": "assistant", "content":  "SV"},

            {"role": "user", "content": "'Please take a picture of the scene'"}, 
            {"role": "assistant", "content":  "TP"},

            {"role": "user", "content": "'something not on the list of options or not valid has been said'"} ,
            {"role": "assistant", "content":  "NV"},

            #This is the acutial question to the chatgpt.
            {"role": "user", "content": prompt}    
        ]
    )
        
    #extract the index. The index is a two character string
    index = completions['choices'][0]['message']['content']

    #check to make sure that it is only 2 characters...otherwise, GPT may have returened a novel ;-)
    if (index in listofpossiblecommands) and len(index) == 2:
        print ("=====>")
        print(index) 
        return (choices [index])
    else:
        print("didn't understand")
        return (choices["NV"]) # NV is an index 

################################################################

# # Load Whisper model
whisper_model = whisper.load_model("base") # You can also use "tiny", "small", etc.

#Audio settings
samplerate = 16000
duration = 5  # seconds

# Main loop
try:
    print("#" * 80)
    print("Listening for voice commands. Press Ctrl+C to stop.")
    print("#" * 80)

    while True:
        # Record audio
        audio = sd.rec(int(samplerate * duration), samplerate=samplerate, channels=1, dtype='float32')
        sd.wait()

        # Save to temporary WAV file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            sf.write(f.name, audio, samplerate)
            audio_path = f.name

        # Transcribe using Whisper
        result = whisper_model.transcribe(audio_path)
        prompt = result['text'].strip()

        if not prompt:
            continue

        print("You said:", prompt)
        cmd = AskGPT(prompt)

        # ROS command logic
        if cmd == "davinci start":
            print("Running autocamera")
            for topic in ['/assistant/clutch_and_move/run', '/assistant/joystick/run', '/assistant/oculus/run', '/assistant/clutchless/run']:
                rospy.Publisher(topic, Bool, latch=True, queue_size=1).publish(Bool(False))
            run_pub.publish(True)
            voiceCommand_pub.publish("Da Vinci Start")
            playsound('sound95.wav')

        elif cmd == "davinci stop":
            print("Stopping autocamera")
            run_pub.publish(False)
            voiceCommand_pub.publish("Da Vinci Stop")
            playsound('sound95.wav')

        elif cmd == "davinci track right":
            print("Tracking right")
            track_pub.publish("right")
            voiceCommand_pub.publish("Da Vinci track right")
            playsound('sound95.wav')

        elif cmd == "davinci track left":
            print("Tracking left")
            track_pub.publish("left")
            voiceCommand_pub.publish("Da Vinci track left")
            playsound('sound95.wav')

        elif cmd == "davinci track middle":
            print("Tracking middle")
            track_pub.publish("middle")
            voiceCommand_pub.publish("Da Vinci track middle")
            playsound('sound95.wav')

        elif cmd == "davinci keep right":
            print("Keeping right")
            keep_pub.publish("right")
            voiceCommand_pub.publish("Da Vinci keep right")
            playsound('sound95.wav')

        elif cmd == "davinci keep left":
            print("Keeping left")
            keep_pub.publish("left")
            voiceCommand_pub.publish("Da Vinci keep left")
            playsound('sound95.wav')

        elif cmd == "davinci keep middle":
            print("Keeping middle")
            keep_pub.publish("middle")
            voiceCommand_pub.publish("Da Vinci keep middle")
            playsound('sound95.wav')

        elif cmd == "davinci find tools":
            print("Finding tools")
            findtools_pub.publish()
            voiceCommand_pub.publish("Da Vinci find my tools")
            playsound('sound95.wav')

        elif cmd == "davinci take picture":
            print("Taking picture")
            picture_pub.publish(True)
            voiceCommand_pub.publish("Da Vinci take picture")
            playsound('sound95.wav')
            picture_pub.publish(False)

        elif cmd == "davinci start video":
            print("Starting video")
            video_pub.publish(True)
            voiceCommand_pub.publish("Da Vinci begin recording")
            playsound('sound95.wav')

        elif cmd == "davinci stop video":
            print("Stopping video")
            video_pub.publish(False)
            voiceCommand_pub.publish("Da Vinci end recording")
            playsound('sound95.wav')

        elif cmd == "Something not valid":
            print("Not valid")
            playsound('repeat.wav')

except KeyboardInterrupt:
    print("\nExiting...")

except Exception as e:
    print("Error:", str(e))

