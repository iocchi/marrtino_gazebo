import os, time
import threading
import json
import openai
import chromadb
import queue
import re
import asyncio

from messages import getMessageDispatcher

ai_G_connections = None
ai_notify_ai_results = None

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

def ai_set_connections(G_conn, notify_fn):
    global ai_G_connections, ai_notify_ai_results

    ai_G_connections = G_conn
    ai_notify_ai_results = notify_fn


def notify_ai_results(content):
    ai_notify_ai_results(content)




def detect_ai_provider(key_string):
    if not isinstance(key_string, str) or not key_string:
        return "Invalid Input"

    # Define the signature patterns
    patterns = {
        "OpenAI": r"^(sk-proj-[a-zA-Z0-9-]{40,}|sk-[a-zA-Z0-9]{32,})$",
        "Anthropic": r"^sk-ant-api\d{2}-[a-zA-Z0-9\-_]{80,}$",
        "Google Gemini": r"^AIzaSy[a-zA-Z0-9\-_]{33}$"
    }

    for provider, pattern in patterns.items():
        if re.match(pattern, key_string):
            return provider

    # Fallback: Simple prefix check if the regex is too strict
    if key_string.startswith("sk-ant-"): return "Anthropic"
    if key_string.startswith("sk-"):     return "OpenAI"
    if key_string.startswith("AIza"):    return "Google"

    return "Unknown Provider"


class AI:
    def __init__(self):
        self.gpt_total_tokens = 0
        self.gpt_input_tokens = 0
        self.gpt_output_tokens = 0

        self.api_key = None
        self.client = None
        try:
            api_key = os.getenv("OPENAI_API_KEY")
            if api_key is None or api_key=="":
                with open("key.txt", "r") as f:
                    api_key = f.read()
        except Exception as e:
            print(f"AI Error reading OPENAI key: {e}")
        if api_key is not None and api_key!="":
            self.setkey(api_key)

        self.init_chromadb()

        self.logf = open(os.path.join(self.get_user_path(), "ai.log"), "a")

    def __del__(self):
        print(f"Used tokens: input {self.gpt_input_tokens} output {self.gpt_output_tokens} | Est. cost: {(self.gpt_input_tokens*MODEL_cost_input+self.gpt_output_tokens*MODEL_cost_output)/1e6} USD")
        self.logf.close()

    def get_user_path(self):
        upath = os.getenv('PATH_USERS')
        if upath is None:
            self.get_logger().warn(f"PATH_USERS env not found. Using '/op/users'")
            upath = '/opt/users'

        cuf = os.path.join(upath,"current_user")
        try:
            with open(cuf, "r") as f:
                l = f.readline().strip()
        except:
            l = ''
        if l == '':
            l = 'anonymous'

        userpath = os.path.join(upath, l)
        if not os.path.isdir(userpath):
            os.mkdir(userpath)

        return userpath

    def setkey(self, key):

        self.api_key = ''
        key = key.strip()
        prov = detect_ai_provider(key)
        if prov=="OpenAI":
            self.api_key = key.strip()
        elif prov in ["Anthropic","Google"]:
            self.log_write(f"AI ERROR: {prov} not yet supported")
        else:
            # read secrets from file

            print(f"read {key} from secrets...")

            try:
                upath = os.getenv('PATH_USERS')
                if upath is None:
                    self.get_logger().warn(f"PATH_USERS env not found. Using '/op/users'")
                    upath = '/opt/users'
                secfile = os.path.join(upath, "aisecrets.txt")
                with open(secfile, 'r', encoding='utf-8') as file:
                    data = json.load(file)
                    if key in data.keys():
                        self.api_key = data[key].strip()
            except FileNotFoundError:
                print("The file was not found.")
            except json.JSONDecodeError:
                print("The file contains invalid JSON.")

            if key!='':
                self.log_write(f"AI ERROR: Unknown key {key}")

        if self.api_key is None or self.api_key == '':
            self.client = None
            print("OpenAI client OFF")
        else:
            self.client = openai.OpenAI(api_key = self.api_key)
            print("OpenAI client ON")



    def init_chromadb(self):
        
        self.clientdb = chromadb.PersistentClient(path=os.path.join(self.get_user_path(), "kb"))

        # reset
        #self.clientdb.delete_collection(name="memory")

        self.memory = self.clientdb.get_or_create_collection(name="memory")

        all_ids = self.memory.get()['ids']
        
        if all_ids:
            self.memoryid = int(all_ids[-1])   # last id for incremental add
        else:
            self.memoryid = 0


    def log_write(self, content):
        self.logf.write(content+"\n")
        self.logf.flush()
        # print(f"*** {notify_ai_results} ***")
        try:
            notify_ai_results(content)  # OK when executed from exec in codeserver
        except Exception as e:
            print(f"AI log_write: cannot notify ai results {content}")
            print(e)

    

    def send_llm_messages(self, messages):
        if self.api_key is None or self.api_key == '' or self.client is None:
            self.log_write("AI ERROR: send_llm_messages: OpenAI key missing")
            return ''

        try:
            response = self.client.responses.create(
                model = MODEL,
                input = messages,
                # temperature = 0.5,
            )
        except openai.AuthenticationError:
            self.log_write("AI ERROR: AuthenticationError: OpenAI key valid?")
            return ''
        except openai.BadRequestError as e:
            self.log_write(f"AI ERROR: Bad request {e}")
            return ''
        except Exception as e:
            self.log_write(f"AI ERROR: {e}")
            return ''
        
        self.gpt_total_tokens += response.usage.total_tokens
        self.gpt_input_tokens += response.usage.input_tokens
        self.gpt_output_tokens += response.usage.output_tokens
        
        self.log_write(f"{messages[-1]['content']}\n{response.output_text}\n{response.usage.input_tokens};{response.usage.output_tokens}\n----\n")

        return response.output_text
        
    # returns chat message for content
    def chat(self, content):
        user_message = {
            'role': 'user',
            'content': content,
        }
        self.log_write("chat")
        response = self.send_llm_messages([chat_system, user_message])
        return response


    # returns python code for content 
    def code(self, content):
        user_message = {
            'role': 'user',
            'content': content,
        }
        self.log_write("code")
        response = self.send_llm_messages([code_system, user_message])
        return response


    def vision(self, image_b64, prompt):
        if self.api_key is None or self.api_key == '' or self.client is None:
            self.log_write("AI ERROR: vision: OpenAI key missing")
            return ''

        # img must be a base64 encoding of the image !!!
        try:           
            response = self.client.responses.create(
                model=MODEL,
                input=[
                    {
                    "role": "user",
                    "content": [
                        {
                          "type": "input_text",
                          "text":f"{prompt}"
                        },
                        {
                          "type": "input_image",
                          "image_url": f"data:image/jpeg;base64,{image_b64}"
                        },
                    ],
                    }
                ],
                # temperature=0.1,
                )

            self.gpt_total_tokens += response.usage.total_tokens
            self.gpt_input_tokens += response.usage.input_tokens
            self.gpt_output_tokens += response.usage.output_tokens

            self.log_write(f"vision\n{prompt}\n{response.output_text}\n{response.usage.input_tokens};{response.usage.output_tokens}\n----\n")
            
            return response.output_text
        except Exception as e:
            print(f"AI vision:: Getting response - {e}")
            return None


    def vision_file(self, img_filepath, prompt):

        image_b64 = None

        # Read current image with the view of the robot
        try:
            with open(img_filepath, "rb") as img_file:
                encoded_image = base64.b64encode(img_file.read()).decode("utf-8")
        except Exception as e:
            print(f"AI vision:: Reading image {img_filepath} - {e}")
            return None

        if image_b64 is None:
            return None
        
        return self.vision(image_b64, prompt)

    # query a description
    def query(self, description, query):
        if self.api_key is None or self.api_key == '' or self.client is None:
            print("AI ERROR: query: OpenAI key missing")
            return ''

        query_system = { 
            'role': 'system',
            'content': "Answer the user request about this description: " + description
        }
        user_message = {
            'role': 'user',
            'content': query,
        }
        self.log_write(f"query\n{description}\n")        
        response = self.send_llm_messages([query_system, user_message])
        return response


    # store a content in the vector db
    def memorize(self, content):
        self.memoryid += 1    
        self.memory.add(
            documents=[content],
            ids=f"{self.memoryid}"
        )


    # retrieve content from docs in vector db
    def retrieve(self, content, max_dist=2.0, n_results=3):
        results = self.memory.query(
            query_texts=[content],
            n_results=n_results
        )

        docs = results['documents'][0]
        dists = results['distances'][0]

        r = ""
        for doc,dist in zip(docs, dists):
            if dist < max_dist:
                r += doc + ", "

        user_message = {
            'role': 'user',
            'content': content,
        }

        self.log_write(f"retrieve\n{docs}\n{dists}\n{r if r!= '' else '*** nothing ***'}\n")

        if r != '':
            sys_pr = "Answer the user request according to these facts: " + r
            remember_system = {
                'role': 'system',
                'content': sys_pr
            }
        else:
            sys_pr = "Answer the user that there are no information available in your memory"
            remember_system = {
                'role': 'system',
                'content': sys_pr
            }

        response = self.send_llm_messages([remember_system, user_message])
        return response


    def forget(self, ids):        
        self.memory.delete(ids=ids)


    def clear_memory(self):
        all_ids = self.memory.get()['ids']
        if all_ids:
            self.memory.delete(ids=all_ids)
            self.memoryid = 0
            print(f"Memory: deleted {len(all_ids)} documents.")


    def print_memory(self):
        all_data = self.memory.get()
        for (a,b) in zip(all_data['ids'], all_data['documents']):
            print(f"{a}\t| {b}")
  
    def list_memory(self):
        r = ''
        all_data = self.memory.get()
        for (a,b) in zip(all_data['ids'], all_data['documents']):
            r += f"{a}\t| {b}\n"
        return r
  
    

def analyze(response):
    try:
        jr = json.loads(response)
        answer = jr['text']
        code = jr['code']
    except json.decoder.JSONDecodeError as e:
        print(f"Error in JSON format !!!\n{response}\n{e}\n")
        return '',''

    return answer, code


AIagents = []

def cleanAIagents():
    global AIagents
    for ag in AIagents:
        ag.del_listener()
    for ag in AIagents:
        del ag
    AIagents = []

class AIAgent():
    def __init__(self, name, main_system_prompt='', ai=None):
        self.name = name
        self.main_system_prompt = main_system_prompt
        self.system_prompt = { 'role': 'system', 'content': self.main_system_prompt }
        if ai is not None:
            self.ai = ai
        else:
            self.ai = AI()  # assumes OPENAI key is readable in 'key.txt' in this folder
        self.listener = None
        self.cb_fn = None
        self.thr_listen = None
        self.enabled = False
        global AIagents
        AIagents.append(self)

    def __del__(self):
        global AIagents
        AIagents.remove(self)

    def set_system(self, sysprompt):
        self.system_prompt['content'] = self.main_system_prompt + " " + sysprompt

    def add_system(self, sysprompt):
        self.system_prompt['content'] += " " + sysprompt 

    def get_system(self):
        return self.system_prompt['content']

    def askllm(self, content):
        user_message = {
            'role': 'user',
            'content': content,
        }
        self.ai.log_write(f"{self.name}")
        response = self.ai.send_llm_messages([self.system_prompt, user_message])
        return response        

    def add_listener(self, topic, cb_fn, robot, gz):
        # only one listener
        if self.cb_fn is None:
            md = getMessageDispatcher()
            self.listener = md.subscriber(self.name, topic)
            self.cb_fn = cb_fn
            self.thr_listen = threading.Thread(target=self.listener_thread, args=(robot, gz), daemon=True)
            self.thr_listen.start()

    def del_listener(self):
        # only one listener
        if self.cb_fn is not None:
            print(f"AI agent {self.name} delete listener ...")
            self.listener = None
            self.thr_listen.join()
            self.thr_listen = None
            self.cb_fn = None

    def enable(self, val=True):
        self.enabled = val

    def disable(self):
        self.enabled = False

    def listener_thread(self, robot, gz):
        print(f"AI agent {self.name} listen thread started ...")
        while self.listener is not None:
            #print(f"AI agent {self.name} waiting for message ...")
            content = None
            try:
                if self.enabled:
                    print(f">>> AI agent {self.name} listening ...")
                content = self.listener.receive(timeout=3) # blocking
                if self.enabled:
                    print(f"AI agent {self.name} received speech: {content}")
                else:
                    content = None
            except queue.Empty:
                pass
            except Exception as e:
                if e.strip() != '':
                    print(f"Error in listener {self.name} receive\n{e}")
                    self.ai.log_write(f"Error in listener {self.name} receive\n{e}")
                    
            if content is not None:
                response = self.askllm(content)
                print(f">>> listener {self.name} -> {response}")
                text, code = analyze(response)
                if self.cb_fn is not None:
                    self.cb_fn(text)
                if code != '':
                    # print(f">>>> exec\n{code}\n>>>>")
                    try:
                        exec(code, {"robot": robot, "gz": gz})
                    except Exception as e:
                        print(f"Error in listener {self.name} exec\n{e}") 
                        self.ai.log_write(f"Error in listener {self.name} exec\n{e}") 
                self.response_sent = True
                #print(f"AI agent {self.name} response {self.response_sent}")
        print(f"AI agent {self.name} listen thread terminated ...")          

    # blocking until a response is sent or timeout
    # return True if response sent, False if timeout
    def waitfor_response(self, timeout=3600): 
        self.response_sent = False
        while self.enabled and not self.response_sent and timeout>0:
            d = 0.5
            time.sleep(d)
            timeout -= d
        return self.response_sent

