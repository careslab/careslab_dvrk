#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import openai
import os
openai.api_key = os.environ['OPENAI_API_KEY'] # Set your OpenAI API key as an environment variable
class ChatGPTNode:

    def __init__(self):
        rospy.init_node('chatgpt_node')
        self.command_sub = rospy.Subscriber('chatgpt/command', String, self.process_command)
        self.response_pub = rospy.Publisher('chatgpt/response', String, queue_size=10)
        self.model_engine = "davinci" # Change this to your preferred OpenAI model engine
        self.temperature = 0.7 # Change this to your preferred temperature for generating responses

    def process_command(self, command):
        prompt = command.data
        response = self.generate_response(prompt)
        self.response_pub.publish(response)

    def generate_response(self, prompt):
        completions = openai.Completion.create(
            engine=self.model_engine,
            prompt=prompt,
            max_tokens=1024,
            n=1,
            stop=None,
            temperature=self.temperature,
        )
        message = completions.choices[0].text.strip()
        return message

if __name__ == '__main__':
    chatgpt_node = ChatGPTNode()
    rospy.spin()