import asyncio
import websockets
import json
import threading

#from websockets.asyncio.client import connect
from websockets.sync.client import connect
from ollama import chat, ChatResponse
from datetime import datetime
import pytz

websocket = None

chat_logfile = open('smarrtino_chat.log', 'a')
nlprog_logfile = open('smarrtino_nlprog.log', 'a')

# MODEL='llama3'

MODEL='codellama:7b'

chat_messages = [ {
    'role': 'system',
    'content': "You are a small educational mobile robot named SMARRtino. You have two driving wheels and one caster wheel for stability. You can move forward, backward, and turn left and right. You have two arms that you can raise up and down. You cannot grasp or manipulate objects, because you do not have hands or grippers. You have a head that you can move to look up, down, left and right. You can speak and listen. You can understand natural language and reply to users. Answer user requesta. Always use short answers."
} ]


nlprog_messages = [ {
    'role': 'system',
    'content': "You are a small educational mobile robot named SMARRtino. You have two driving wheels and a caster wheel, two arms (left and right arm), and a pan-tilt head. You can move on a planar surface by using the following Python high-level functions: 'robot.forward(m)': move ahead by m meters. If the user does not specify any distance, move by 1 meter. If you are asked to turn left, use 'robot.turn(90)' or 'robot.left(90)' to turn left by 90 degrees. Instead, when you are asked to turn right, use 'robot.turn(-90)' or 'robot.right(90)' to turn right by 90 degrees. Do not make mistakes with left and right. Always double check that the function corresponds with the user request. For example, to turn left, then turn right use 'robot.turn(90)' and then 'robot.turn(-90)'.  For example, to turn around, use 'robot.turn(180)'. To raise up your left arm above your head, use 'robot.left_arm(180)'. For raising up the right arm use 'robot.right_arm(180)'. To move arms in front of you use 'robot.left_arm(90)' for the left arm and 'robot.right_arm(90)' for the right arm. To place arms in the rest position down, use 'robot.left_arm(0)' for the left arm and 'robot.right_arm(0)' for the right arm. The head can turn left with the function 'robot.pan(90)' and right with 'robot.pan(-90)'. You can move the head up with 'robot.tilt(30)' and down with 'robot.tilt(-30)'. When the user asks to look somewhere, use head movements. For example, for looking straight ahead use 'robot.pan(0)' and 'robot.tilt(0)', to look left use 'robot.pan(90)', to look right use 'robot.pan(-90)'. When the user asks to turn, use only the functions 'robot.turn', 'robot.left', 'robot.right'. When the user asks to look somewhere, use only the function 'robot.pan'. When the user asks to execute multiple commands, use Python functions in a sequence. For example, if the user asks to move forward and then turn left, use the sequence of functions 'robot.forward(1)\nrobot.turn(90)'. If the user asks to look forward and put the arms down, use the sequence 'robot.pan(0)\nrobot.tilt(0)\nrobot.left_arm(0)\nrobot.right_arm(0)'.  Think step-by-step and always format the response with Python functions formatted between tags <CODE> </CODE>. Use new line character to separate Python functions in the <CODE> tag. Do not add spaces or other characters in the <CODE> tags. Add Python comments explaining the meaning of any instruction in the code."
} ]




chat_logfile.write(f"{datetime.now(pytz.timezone('Europe/Rome')).strftime('%y/%m/%d:%H:%M:%S')}\n")
chat_logfile.write("system: "+chat_messages[0]['content']+"\n\n")
chat_logfile.flush()

nlprog_logfile.write(f"{datetime.now(pytz.timezone('Europe/Rome')).strftime('%y/%m/%d:%H:%M:%S')}\n")
nlprog_logfile.write("system: "+nlprog_messages[0]['content']+"\n\n")
nlprog_logfile.flush()

def chat_step(content):
    global chat_messages, chat_logfile

    chat_messages.append({
        'role': 'user',
        'content': content,
      })

    chat_logfile.write(f"{datetime.now(pytz.timezone('Europe/Rome')).strftime('%y/%m/%d:%H:%M:%S')}\n")
    chat_logfile.write("user:\n"+content+"\n\n")
    chat_logfile.flush()

    response: ChatResponse = chat(model=MODEL, messages=chat_messages)

    #print("Chat: ")
    #print(response)

    chat_messages.append(response.message)

    chat_logfile.write(f"{datetime.now(pytz.timezone('Europe/Rome')).strftime('%y/%m/%d:%H:%M:%S')}\n")
    chat_logfile.write("assistant:\n"+response.message.content+"\n\n")
    chat_logfile.flush()

    return response.message.content



def nlprog_step(content):
    global nlprog_messages

    user_content = "Write the Python code to execute this command: " + content + ". Format the output in <CODE></CODE> tags, with new-line separator. Do not put any space between the tag '<CODE>' and the Python function starting with 'robot'."

    nlprog_messages.append({
        'role': 'user',
        'content': user_content,
      })

    nlprog_logfile.write(f"{datetime.now(pytz.timezone('Europe/Rome')).strftime('%y/%m/%d:%H:%M:%S')}\n")
    nlprog_logfile.write("user:\n"+user_content+"\n\n")
    nlprog_logfile.flush()

    response: ChatResponse = chat(model=MODEL, messages=nlprog_messages)

    #print("NLprog: ")
    #print(response)

    nlprog_messages.append(response.message)

    nlprog_logfile.write(f"{datetime.now(pytz.timezone('Europe/Rome')).strftime('%y/%m/%d:%H:%M:%S')}\n")
    nlprog_logfile.write("assistant:\n"+response.message.content+"\n\n")
    nlprog_logfile.flush()

    return response.message.content



async def echo(websocket):
    print(f"Client connected from {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"Received message from client: {message}")
            try:
                data = json.loads(message)
                if "chat" in data:
                    received_text = data["chat"]
                    print(f"Chat:\n{received_text}")
                    response = chat_step(received_text)
                    await websocket.send(json.dumps({"status": "say", "message": response}))
                elif "nlprog" in data:
                    received_text = data["nlprog"]
                    print(f"NLProg:\n{received_text}")
                    response = nlprog_step(received_text)
                    await websocket.send(json.dumps({"status": "llmcode", "message": response}))

            except json.JSONDecodeError:
                print(f"Received non-JSON message: {message}")
                await websocket.send(json.dumps({"status": "error", "message": "Expected JSON format."}))

    except websockets.exceptions.ConnectionClosedOK:
        print(f"Client {websocket.remote_address} disconnected normally.")
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Client {websocket.remote_address} disconnected with error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print(f"Client {websocket.remote_address} connection closed.")





async def main():
    # Start the WebSocket server on host, port 8766
    async with websockets.serve(echo, "0.0.0.0", 8766) as server:
        print("WebSocket server started on ws://localhost:8766")
        await server.serve_forever()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())

