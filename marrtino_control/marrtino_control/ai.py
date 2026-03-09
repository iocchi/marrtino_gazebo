import os, time
import threading
import openai
import chromadb

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

        self.api_key = None
        self.client = None
        try:
            self.api_key = os.getenv("OPENAI_API_KEY")
            if self.api_key is None or self.api_key=="":
                with open("key.txt", "r") as f:
                    self.api_key = f.read()
        except Exception as e:
            print(f"AI Error reading OPENAI key: {e}")
        if self.api_key is not None and self.api_key!="":
            self.api_key = self.api_key.strip()
            self.client = openai.OpenAI(api_key = self.api_key)
            print("AI: OPENAI client OK")
        else:
            print("AI: OPENAI client not ready!")

        self.init_chromadb()

        self.logf = open("ai.log", "a")

    def __del__(self):
        print(f"Used tokens: input {self.gpt_input_tokens} output {self.gpt_output_tokens} | Est. cost: {(self.gpt_input_tokens*MODEL_cost_input+self.gpt_output_tokens*MODEL_cost_output)/1e6} USD")
        self.logf.close()

    def setkey(self, key):
        key = key.strip()
        if key != self.api_key:
            self.api_key = key
            self.client = openai.OpenAI(api_key = self.api_key)


    def init_chromadb(self):
        
        self.clientdb = chromadb.PersistentClient(path="./kb")

        # reset
        #self.clientdb.delete_collection(name="memory")

        self.memory = self.clientdb.get_or_create_collection(name="memory")

        all_ids = self.memory.get()['ids']
        
        if all_ids:
            self.memoryid = int(all_ids[-1])   # last id for incremental add
        else:
            self.memoryid = 0


    def log_write(self, content):
        self.logf.write(content)
        self.logf.flush()


    def send_llm_messages(self, messages):
        if self.api_key is None or self.api_key == '' or self.client is None:
            print("AI: OpenAI key missing")
            return ''

        try:
            response = self.client.responses.create(
                model = MODEL,
                input = messages,
                # temperature = 0.5,
            )
        except openai.AuthenticationError:
            print("AI AuthenticationError: OpenAI key valid?")
            return ''
        
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


    def vision(self, image_b64, prompt):
        if self.api_key is None or self.api_key == '' or self.client is None:
            print("AI: OpenAI key missing")
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

            self.logf.write("vision\n")
            self.logf.write(f"{prompt}\n{response.output_text}\n{response.usage.input_tokens};{response.usage.output_tokens}\n----\n")
            self.logf.flush()

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
            print("AI: OpenAI key missing")
            return ''

        query_system = { 
            'role': 'system',
            'content': "Answer the user request about this description: " + description
        }
        user_message = {
            'role': 'user',
            'content': query,
        }
        self.logf.write("query\n")
        self.logf.write(f"{description}\n")        
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
    def retrieve(self, content, max_dist=1.0, n_results=3):
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

        self.logf.write("retrieve\n")
        self.logf.write(f"{docs}\n{dists}\n{r if r!= '' else '*** nothing ***'}\n")

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
  
    




AIagents = []

def cleanAIagents():
    global AIagents
    for ag in AIagents:
        ag.del_listener()
    for ag in AIagents:
        del ag
    AIagents = []

class AIAgent():
    def __init__(self, name, system_prompt):
        self.name = name
        self.system_prompt = system_prompt
        self.ai = AI()
        self.listener = None
        self.cb_fn = None
        self.thr_listen = None
        global AIagents
        AIagents.append(self)

    def __del__(self):
        global AIagents
        AIagents.remove(self)

    def add_system(self, sysprompt):
        self.system_prompt['content'] += sysprompt 

    def askllm(self, content):
        user_message = {
            'role': 'user',
            'content': content,
        }
        self.ai.logf.write(f"{self.name}\n")
        response = self.ai.send_llm_messages([self.system_prompt, user_message])
        return response        

    def add_listener(self, md, topic, cb_fn):
        # only one listener
        if self.cb_fn is None:
            self.listener = md.subscriber(self.name, topic)
            self.cb_fn = cb_fn
            self.thr_listen = threading.Thread(target=self.listener_thread, args=(), daemon=True)
            self.thr_listen.start()

    def del_listener(self):
        # only one listener
        if self.cb_fn is not None:
            self.listener = None
            self.thr_listen.join()
            self.thr_listen = None
            self.cb_fn = None

    def listener_thread(self):
        print(f"AI agent {self.name} listen thread started ...")
        while self.listener is not None:
            #print(f"AI agent {self.name} waiting for message ...")
            try:
                content = self.listener.receive(timeout=3) # blocking
                print(f"AI agent {self.name} received speech: {content}")
            except:
                content = None
            if self.cb_fn is not None and content is not None:
                response = self.askllm(content)
                self.cb_fn(response)
                self.response_sent = True
                #print(f"AI agent {self.name} response {self.response_sent}")
        print(f"AI agent {self.name} listen thread terminated ...")          

    # blocking until a response is sent or timeout
    # return True if response sent, False if timeout
    def waitfor_response(self, timeout=3600): 
        self.response_sent = False
        while not self.response_sent and timeout>0:
            d = 0.5
            time.sleep(d)
            timeout -= d
        return self.response_sent

