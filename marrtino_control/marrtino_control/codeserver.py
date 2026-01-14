import os
import sys
import asyncio
import websockets
import json
import threading
import time
import math, random
import traceback
from thread2 import Thread
import datetime
import requests

from control import MARRtinoController

sys.path.append("../../marrtino_gazebo/src")
from gz_models import ModelManager


# config variables

HTTP_PORT = 3080
WS_PORT = 9876

robot = None
gz_models = None

code_running = None
websocket = None

run_code_thread = None

simulation_run = False  # simulation is running
keepalive = True  # keep alive when client disconnects


# SRL connection
SRL_SERVICE = 'http://10.96.0.2:5000'

def safe_fetch_json(url, method='GET'):
    """Esegue una richiesta HTTP sincrona al servizio SRL."""
    try:
        response = requests.request(method, f"{SRL_SERVICE}/{url}", timeout=5)
        if response.status_code == 200:
            return response.json()
        print(f"API Error fetching {url}: Status {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Request failed for {url}: {e}")
    return None

# log file
flog = None

def printt(s):
    global flog
    if flog==None:
        flog = open("codeserver.log" , "a", encoding='utf-8')
    #s = s.decode('utf-8')
    
    t = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    flog.write("%s;%s\n" %(t,s))
    flog.flush()
    print(s)


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


# global
nconnections = 0


async def handler(websocket):
    global code_running, robot, gz_models, simulation_run, keepalive
    global nconnections

    clientIP = websocket.request_headers.get('X-Real-Ip', '')
    printt(f"Client connected from {clientIP} id: {websocket.id}")
    
    nconnections += 1
    print(f"Connected clients: {nconnections}")
    #await websocket.send(f"USER IP {clientIP}")
    await websocket.send(
        json.dumps({ "status": "user",
                     "name": "Unknown", 
                     "id": "noID", 
                     "ip": clientIP } ))

    lsb_user = False

    try:
        j = safe_fetch_json("api/service/inlab")
        if j is not None:
            print(f"inlab {j}")
        j = safe_fetch_json(f"api/user/by-ip/{clientIP}")
        if j is not None and j['user'] is not None:
            print(f"user {j}")

            firstname = j['user']['first_name']
            lastname = j['user']['last_name']
            email = j['user']['email']
            userid = j['user']['id']
            if 'studenti.uniroma1.it' in email:
                uname = email.split('@')[0]
                userid = uname.split('.')[1]

            lsb_user = True

            printt(f"LSB Connected {clientIP} {firstname} {lastname} {email} {userid}")
            # await websocket.send(f"USER {firstname} {lastname} {userid} IP {clientIP}")
            await websocket.send(
                json.dumps({ "status": "user",
                             "name": f"{firstname} {lastname}", 
                             "id": userid, 
                             "ip": clientIP } ))

    except Exception as e:
        print("Error in accessing SRL services")
        print(e)

    #os.system("cp noimage.jpg lastimage.png")
    if not simulation_run: # start simulation
        gz_pause(False)   # unpause simulation
        gz_gui(True)   # start gz gui
        simulation_run = True
    try:
        async for message in websocket:
            if message == '__ping__':
                #print(f"{clientIP} {message}")
                continue
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

                # full code
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

                # single robot function
                elif "robotfn" in data:
                    received_code = data["robotfn"]
                    while code_running is not None:
                        print("Waiting for code execution to be available....")
                        time.sleep(0.5)

                    print(f"Robot function: {received_code}")

                    r = None
                    try:
                        r = eval(received_code)
                    except Exception as e:
                        print(f"Error: {e}")
                        await websocket.send(json.dumps({"status": "error", "message": f"{e}", "disable_send": "false"}))

                    print(f"Robot function result: [{r}]")
                    await websocket.send(json.dumps({"status": "robotfn_done", "result": r}))

                elif "gazebo" in data:
                    gz_fn = data["gazebo"]  # should be gz_models.fn(...) 

                    print(f"Gazebo function: {gz_fn}")

                    r = None
                    try:
                        r = eval(gz_fn)
                    except Exception as e:
                        print(f"Error: {e}")
                        await websocket.send(json.dumps({"status": "error", "message": f"{e}", "disable_send": "false"}))

                    print(f"Gazebo function result: [{r}]")
                    await websocket.send(json.dumps({"status": "gazebo_done", "result": r}))


                elif "keepalive" in data:
                    # keepalive = data["keepalive"] == "True"
                    pass

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

        print(f"Connection closed for {clientIP}")
        #if lsb_user:
        #    url = f"api/user/{clientIP}/disconnect"
        #    method = 'PUT'
        #    safe_fetch_json(url, method)
        #    print(f"LSB Disconnected {clientIP}")
        await websocket.close()

    except websockets.exceptions.ConnectionClosedOK:
        printt(f"Client {clientIP} disconnected normally.")
    except websockets.exceptions.ConnectionClosedError as e:
        printt(f"Client {clientIP} disconnected with error: {e}")
    except Exception as e:
        printt(f"An unexpected error occurred: {e}")
    finally:
        printt(f"Finally client {clientIP} connection closed.")
        nconnections -= 1
        print(f"Connected clients: {nconnections}")

        #os.system("cp noimage.jpg lastimage.png")
        if simulation_run and not keepalive: # stop simulation
            gz_pause(True)   # pause simulation
            gz_gui(False)    # kill gz gui
            simulation_run = False

async def main():
    global robot, gz_models
    
    gz_pause(False)   # need clock to start the robot
    robot = MARRtinoController()
    gz_models = ModelManager()
    gz_pause(True)

    # Start the WebSocket server on server host
    async with websockets.serve(handler, "0.0.0.0", WS_PORT, ping_timeout=60) as server:
        print(f"WebSocket server started on ws://0.0.0.0:{WS_PORT}")
        await server.serve_forever()  # Run forever

'''
import http.server
import socketserver

def run_http_server():

    # NOTE conflict with path in control.py to save last_image
    # WWW_PATH = '../../www'
    # chdir to www
    #os.chdir(WWW_PATH)


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
'''

if __name__ == "__main__":

    # not needed when nginx is running
    #server_thread = threading.Thread(target=run_http_server, daemon=True)
    #server_thread.start()

    asyncio.run(main())

