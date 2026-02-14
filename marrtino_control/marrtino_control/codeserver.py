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

# robot object
from control import MARRtinoController

# gz object
sys.path.append("../../marrtino_gazebo/src")
from gz_models import ModelManager

# ai object
from ai import AI

# config variables

HTTP_PORT = 3080
WS_PORT = 9876

robot = None
gz = None
ai = None

code_running = None
websocket = None

run_code_thread = None

simulation_run = False  # simulation is running
keepalive = True  # keep alive when client disconnects

lsb_mode = os.getenv("WGIF") == 'lsb11'
print(f"LSB mode: {lsb_mode}")

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


def notify_robot_say(G_connections):
    global robot
    while True:
        if robot is not None: 
            if robot.simulated_say is not None:
                print(f" -- send websocket {robot.simulated_say}")
                for websocket in G_connections:
                    asyncio.run(send_robot_say(websocket, robot.simulated_say))
                    time.sleep(0.2)
                time.sleep(1)
                robot.simulated_say = None
        time.sleep(0.5)


def run_thread(code, websocket, donotify=True):
    global code_running, run_code_thread
    print("Running code in a thread ...")
    code_running = code
    run_code_thread = Thread(target=run_code, args=(websocket, donotify), daemon=True)
    run_code_thread.start()
    print("Running code in a thread started.")


def run_eval(fn):
    global robot, ai, gz
    r = eval(fn)
    return r


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
    print(f' -- send robot say: {sentence}')
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
G_connections = []
nconnections = 0
current_userid = None

th_robot_say = Thread(target=notify_robot_say, args=(G_connections, ), daemon=True)
th_robot_say.start()


def get_user_path():
    upath = os.getenv('PATH_USERS')
    if upath is None:
        print(f"codeserver: PATH_USERS env not found. Using '/op/users'")
        upath = '/opt/users'
    return upath

async def handler(websocket):
    global code_running, robot, gz, ai, simulation_run, keepalive, lsb_mode
    global nconnections, G_connections, current_userid

    clientIP = websocket.request_headers.get('X-Real-Ip', '')
    printt(f"Client connected from {clientIP} id: {websocket.id}")
    
    nconnections += 1
    G_connections.append(websocket)
    
    print(f"Connected clients: {nconnections}")
    #print(f"WS Connections: {G_connections}")

    userid = clientIP   # default user_id
    firstname = "No"
    lastname = "Name"
    email = "@"
    lsb_user = False

    if lsb_mode:
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


        except Exception as e:
            print("Error in accessing SRL services")
            print(e)

    lsb_str = "LSB " if lsb_user else ''
    printt(f"{lsb_str}User Connected {userid} {firstname} {lastname} {email} {clientIP}")
    t0 = time.time()

    if nconnections == 1:  # current user
        current_userid = userid
        cuf = os.path.join(get_user_path(),"current_user")
        with open(cuf, "w") as f:
            f.write(userid+"\n")

    elif nconnections > 1:
        if current_userid != userid:
            printt("BIG WARNING: multiple users!!! current {current_userid} - now {userid} !!!")

    await websocket.send(
        json.dumps({ "status": "user",
                        "name": f"{firstname} {lastname}", 
                        "id": userid, 
                        "ip": clientIP } ))

    #os.system("cp noimage.jpg lastimage.png")
    if lsb_mode and not simulation_run: # start simulation
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
                    received_code = data["robotfn"]  # should be robot.fn(...) 
                    while code_running is not None:
                        print("Waiting for code execution to be available....")
                        time.sleep(0.5)

                    print(f"Robot function: {received_code}")

                    r = None
                    success = False
                    while not success:
                        try:
                            # r = run_eval(received_code)   -- blocking does not allow ping and causes disconnection !!!
                            r = await asyncio.to_thread(run_eval, received_code)
                            success = True
                        except Exception as e:
                            print(f"Robot Error: {e}")
                            await websocket.send(json.dumps({"status": "error", "message": f"{e}"}))
                            time.sleep(1)

                    if type(r) is str and len(r)>80:
                        print(f"Robot function result: [{r[0:20]}...] len={len(r)}")
                    else:
                        print(f"Robot function result: [{r}]")

                    await websocket.send(json.dumps({"status": "robotfn_done", "result": r}))

                elif "gazebo" in data:
                    gz_fn = data["gazebo"]  # should be gz.fn(...) 

                    print(f"Gazebo function: {gz_fn}")

                    r = None
                    success = False
                    while not success:
                        try:
                            r = eval(gz_fn)
                            success = True
                        except Exception as e:
                            print(f"Gazebo Error: {e}")
                            await websocket.send(json.dumps({"status": "error", "message": f"{e}"}))
                            time.sleep(1)

                    print(f"Gazebo function result: [{r}]")
                    await websocket.send(json.dumps({"status": "gazebo_done", "result": r}))

                elif "ai" in data:
                    ai_fn = data["ai"]  # should be ai.fn(...) 

                    print(f"AI function: {ai_fn}")

                    r = None
                    success = False
                    while not success:
                        try:
                            r = await asyncio.to_thread(run_eval, ai_fn)
                            success = True
                        except Exception as e:
                            print(f"AI Error: {e}")
                            await websocket.send(json.dumps({"status": "error", "message": f"{e}", "disable_send": "false"}))
                            time.sleep(1)

                    print(f"AI function result: [{r}]")
                    await websocket.send(json.dumps({"status": "ai_done", "result": r}))


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

        lsb_str = "LSB " if lsb_user else ''
        printt(f"{lsb_str}User Disconnect {userid} {firstname} {lastname} {email} {clientIP}")
        tend = time.time()
        printt(f"{lsb_str}User Connection Time {userid} {tend-t0:.1f} sec")

        nconnections -= 1
        G_connections.remove(websocket)
        print(f"Connected clients: {nconnections}")
        #print(f"WS Connections: {G_connections}")

        if nconnections == 0:
            cuf = os.path.join(get_user_path(),"current_user")
            with open(cuf, "w") as f:
                f.write("\n")
            current_userid = None

        #os.system("cp noimage.jpg lastimage.png")
        if lsb_mode and simulation_run and not keepalive: # stop simulation
            gz_pause(True)   # pause simulation
            gz_gui(False)    # kill gz gui
            simulation_run = False

async def main():
    global robot, gz, ai, lsb_mode
    
    gz_pause(False)   # need clock to start the robot
    robot = MARRtinoController()
    gz = ModelManager()
    if lsb_mode:
        gz_pause(True)
    ai = AI()

    # Start the WebSocket server on server host
    async with websockets.serve(handler, "0.0.0.0", WS_PORT, ping_interval=30, ping_timeout=10) as server:
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

