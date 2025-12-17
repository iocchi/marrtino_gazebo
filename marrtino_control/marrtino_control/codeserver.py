import os
import asyncio
import websockets
import json
import threading
import time
import math, random
import traceback
from thread2 import Thread
import datetime

from websockets.sync.client import connect

from control import MARRtinoController


# config variables

WWW_PATH = '../../www'
HTTP_PORT = 3080
WS_PORT = 9876

ronot = None

code_running = None
websocket = None

run_code_thread = None

def run_code(websocket, donotify=True):
    global robot
    if code_running is not None:
        robot.user_stop = False
        if 'import' not in code_running:    
            try:
                exec(code_running)
            except Exception:
                print(traceback.format_exc())
        asyncio.run(notify_thread_completed(websocket, donotify=donotify))

def print_robot_say(websocket):
    global robot
    while code_running is not None:
        if robot.simulated_say is not None:
            asyncio.run(send_robot_say(websocket, robot.simulated_say))
        robot.simulated_say = None
        time.sleep(0.5)

def run_thread(code, websocket, donotify=True):
    global code_running, run_code_thread
    print("Running code in a thread ...")
    code_running = code
    run_code_thread = Thread(target=run_code, args=(websocket, donotify))
    run_code_thread.start()
    t2 = Thread(target=print_robot_say, args=(websocket, ))
    t2.start()
    print("Running code in a thread started.")


def terminate_thread():
    global code_running, run_code_thread
    if run_code_thread is not None:
        print("Running code terminating ....")
        run_code_thread.terminate()
        print("Running code terminated.")
        run_code_thread = None
        code_running = None


async def notify_thread_completed(websocket, donotify):
    global code_running
    print('Code execution completed.')
    code_running = None
    if donotify:
        print("Notify termination.")
        # notify termination to JS client websocket
        await websocket.send(json.dumps({"status": "success", "message": "Code completed!", "disable_send": "false"}))


async def send_robot_say(websocket, sentence):
    print(f'send robot say: {sentence}')
    await websocket.send(json.dumps({"status": "say", "message": sentence }))


def gz_pause(flag):
    cmd = "gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 "
    cmd = cmd + ( "--req 'pause: true'" if flag else "--req 'pause: false'" )
    print(cmd)
    os.system(cmd)


def gz_gui(flag):
    if flag:
        cmd = "tmux send-keys -t 0:4 './smarrtino_gui.bash' C-m"
    else:
        cmd = "tmux send-keys -t 0:4 C-c"
    print(cmd)
    os.system(cmd)


async def echo(websocket):
    global code_running, robot
    print(f"Client connected from {websocket.remote_address}")
    os.system("cp noimage.jpg lastimage.png")
    gz_pause(False)   # unpause simulation
    gz_gui(True)   # start gz gui
    try:
        async for message in websocket:
            print(f"Received message from client: {message}")
            try:
                data = json.loads(message)
                if "text" in data:
                    received_text = data["text"]
                    print(f"Text:\n{received_text}")
                    await websocket.send(json.dumps({"status": "success", "message": "Text received!"}))

                elif "syscode" in data:
                    received_code = data["syscode"]
                    if code_running is None:
                        print(f"Python system code:\n{received_code}")
                        # do not send code running message
                        try:
                            run_thread(received_code, websocket, donotify=False)
                        except Exception as e:
                            print(f"Error: {e}")
                            await websocket.send(json.dumps({"status": "error", "message": f"{e}", "disable_send": "false"}))
                    else:
                        print("Another program running. Code discarded.")
                        await websocket.send(json.dumps({"status": "error", "message": "Code already running", "disable_send": "false"}))

                elif "code" in data:
                    received_code = data["code"]
                    while code_running is not None:
                        print("Waiting for code execution to be available....")
                        time.sleep(0.5)

                    print(f"Python code:\n{received_code}")
                    await websocket.send(json.dumps({"status": "success", "message": "Code running ...", "disable_send": "true"}))
                    try:
                        run_thread(received_code, websocket)
                    except Exception as e:
                        print(f"Error: {e}")
                        await websocket.send(json.dumps({"status": "error", "message": f"{e}", "disable_send": "false"}))

                elif "signal" in data:
                    received_signal = data["signal"]
                    print(f"Signal code:\n{received_signal}")
                    if received_signal == 'DONE':
                        print("sending code completed !!!")
                        await websocket.send(json.dumps({"status": "success", "message": "Code completed!", "disable_send": "false"}))
                    elif received_signal == 'STOP':
                        print("sending STOP !!!")
                        await websocket.send(json.dumps({"status": "success", "message": "Code stopped!", "disable_send": "false"}))
                        robot.user_stop = True
                        time.sleep(1)
                        terminate_thread()

                elif "say" in data:
                    received_say = data["say"]
                    print(f"Human say:\n{received_say}")
                    robot.simulated_asr = received_say

                else:
                    print("Received message does not contain known key.")
                    await websocket.send(json.dumps({"status": "error", "message": "Invalid message format."}))
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
        os.system("cp noimage.jpg lastimage.png")
        gz_pause(True)   # pause simulation
        gz_gui(False)    # kill gz gui

async def main():
    global robot
    
    gz_pause(False)   # need clock to start the robot
    robot = MARRtinoController()
    gz_pause(True)

    # Start the WebSocket server on server host
    async with websockets.serve(echo, "0.0.0.0", WS_PORT) as server:
        print(f"WebSocket server started on ws://0.0.0.0:{WS_PORT}")
        await server.serve_forever()  # Run forever


import http.server
import socketserver

def run_http_server():

    try:
        httpd = socketserver.TCPServer(("", HTTP_PORT), http.server.SimpleHTTPRequestHandler)
    except OSError as e:
        print(f"Cannot open HTTP server on port {HTTP_PORT}.")
        exit(1)

    print(f"HTTP server started on http://0.0.0.0:{HTTP_PORT} serving {WWW_PATH}")
    try:
        httpd.serve_forever()  # blocking
    except KeyboardInterrupt:
        print("\nServer HTTP interrupt.")
    except Exception as e:
        print(f"Error in server HTTP: {e}")
    finally:
        httpd.server_close()
        print("Server HTTP closed.")


if __name__ == "__main__":

    # chdir to www
    os.chdir(WWW_PATH)

    # not needed when nginx is running
    #server_thread = threading.Thread(target=run_http_server, daemon=True)
    #server_thread.start()

    asyncio.run(main())

