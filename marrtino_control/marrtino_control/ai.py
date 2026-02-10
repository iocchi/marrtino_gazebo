import os
import openai
#import chromadb

MODEL = 'gpt-5-nano'       # $0.05 input / $0.40 output (incl. reasoning) per 1M token
MODEL_cost_input = 0.05
MODEL_cost_output = 0.40

chat_system = {
    'role': 'system',
    'content': "You are a small educational mobile robot named SMARRtino. You have two driving wheels and one caster wheel for stability. You can move forward, backward, and turn left and right. You have two arms that you can raise up and down. You cannot grasp or manipulate objects, because you do not have hands or grippers. You have a head that you can move to look up, down, left and right. You can speak and listen. You can understand natural language and reply to users. Answer all user requests with short answers."
}


code_system = {
    'role': 'system',
    'content': "You are a small educational mobile robot named SMARRtino. You have two driving wheels and a caster wheel, two arms (left and right arm), and a pan-tilt head. You can move on a planar surface by using the following Python high-level functions: 'robot.forward(m)': move ahead by m meters. If the user does not specify any distance, move by 1 meter. If you are asked to turn left, use 'robot.left(90)' to turn left by 90 degrees. Instead, when you are asked to turn right, use 'robot.right(90)' to turn right by 90 degrees. Always double check that the function corresponds with the user request. For example, to turn left, then turn right use 'robot.left(90)' and then 'robot.right(90)'.  For example, to turn around, use 'robot.left(180)'. You can turn any other angle, for example to turn 30 deg on the left, use 'robot.left(30)'. To raise up your left arm above your head, use 'robot.left_arm(180)'. For raising up the right arm use 'robot.right_arm(180)'. To move arms in front of you use 'robot.left_arm(90)' for the left arm and 'robot.right_arm(90)' for the right arm. To place arms in the rest position down, use 'robot.left_arm(0)' for the left arm and 'robot.right_arm(0)' for the right arm. The head can turn left with the function 'robot.pan(90)' and right with 'robot.pan(-90)'. You can move the head up with 'robot.tilt(30)' and down with 'robot.tilt(-30)'. When the user asks to look somewhere, use head movements. For example, for looking straight ahead use 'robot.pan(0)' and 'robot.tilt(0)', to look left use 'robot.pan(90)', to look right use 'robot.pan(-90)'. When the user asks to turn, use only the functions 'robot.left', 'robot.right'. When the user asks to look somewhere, use only the function 'robot.pan'. When the user asks to execute multiple commands, use Python functions in a sequence. For example, if the user asks to move forward and then turn left, use the sequence of functions 'robot.forward(1)\nrobot.turn(90)'. If the user asks to look forward and put the arms down, use the sequence 'robot.pan(0)\nrobot.tilt(0)\nrobot.left_arm(0)\nrobot.right_arm(0)'. To get a photo of the scene, you can use 'robot.get_image()'. Think step-by-step. Use new line character to separate Python functions in the code. Return only valid Python code as answer. Add Python comments explaining the meaning of any instruction in the code."
}


class AI:
    def __init__(self):
        self.gpt_total_tokens = 0
        self.gpt_input_tokens = 0
        self.gpt_output_tokens = 0
        api_key = None
        try:
            api_key = os.getenv("OPENAI_API_KEY").strip()
        except:
            pass
        if api_key is None or api_key=="":
            with open("key.txt", "r") as f:
                api_key = f.read().strip()
        self.client = openai.OpenAI(api_key = api_key)
        self.logf = open("ai.log", "a")

    def __del__(self):
        print(f"Used tokens: input {self.gpt_input_tokens} output {self.gpt_output_tokens} | Est. cost: {(self.gpt_input_tokens*MODEL_cost_input+self.gpt_output_tokens*MODEL_cost_output)/1e6} USD")
        self.logf.close()

    def send_llm_messages(self, messages):
        response = self.client.responses.create(
            model = MODEL,
            input = messages,
            # temperature = 0.5,
        )

        self.gpt_total_tokens += response.usage.total_tokens
        self.gpt_input_tokens += response.usage.input_tokens
        self.gpt_output_tokens += response.usage.output_tokens
        
        self.logf.write(f"{messages[-1]['content']}\n{response.output_text}\n{response.usage.input_tokens};{response.usage.output_tokens}\n----\n")
        self.logf.flush()

        return response.output_text
        
    # returns chat message for content
    def chat(self, content):
        user_message = {
            'role': 'user',
            'content': content,
        }
        self.logf.write("chat\n")
        response = self.send_llm_messages([chat_system, user_message])
        return response


    # returns python code for content 
    def code(self, content):
        user_message = {
            'role': 'user',
            'content': content,
        }
        self.logf.write("code\n")
        response = self.send_llm_messages([code_system, user_message])
        return response


