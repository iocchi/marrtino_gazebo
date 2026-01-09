import subprocess as sub
import requests
import threading
import time, datetime
import sys, os

SRL_SERVICE = 'http://10.96.0.2:5000'

lsb_ip = '10.112.0.11'   # Lab VPN IP = IP of the machine running this program
lsb_port = 3080         # Lab VPN port = port on which the LSB is running
wg_if = 'lsb11'           # Wire>Guard interface

check_interval = 30     # Check intervale [sec]
timeout_disconnect = 60 # Inactivity threhold [sec]

def inlab():
    url = f"api/service/inlab"
    method='GET'
    try:
        #print(f"{method} {SRL_SERVICE}/{url}")
        response = requests.request(method, f"{SRL_SERVICE}/{url}", timeout=10)
        if response.status_code == 200:
            return response.json()

        t = time.time()
        dt = datetime.datetime.fromtimestamp(t)
        ts = dt.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] ERROR API Error fetching {url}: Status {response.status_code}")
    except requests.exceptions.RequestException as e:
        t = time.time()
        dt = datetime.datetime.fromtimestamp(t)
        ts = dt.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] ERROR Request failed for {url}: {e}")
    return None


def is_user_in_lab(cip):
    # check if user with IP cip is in lab
    uinlab = False
    if users_in_lab is None:
        uinlab = True  # we don't know yet, let's try again later
    else:
        for user in users_in_lab:
            uid = user['id']
            client_ip = user['vpn_ip']
            if client_ip == cip:
                uinlab = True
    return uinlab


def active_bookings():
    url = f"api/service/bookings"
    method='GET'
    try:
        #print(f"{method} {SRL_SERVICE}/{url}")
        response = requests.request(method, f"{SRL_SERVICE}/{url}", timeout=10)
        if response.status_code == 200:
            all_bookings = response.json()
            active_bookings = []
            for b in all_bookings:
                if b['status'] == 'active':
                    active_bookings.append(b)
            return active_bookings
        t = time.time()
        dt = datetime.datetime.fromtimestamp(t)
        ts = dt.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] ERROR API Error fetching {url}: Status {response.status_code}")
    except requests.exceptions.RequestException as e:
        t = time.time()
        dt = datetime.datetime.fromtimestamp(t)
        ts = dt.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] ERROR Request failed for {url}: {e}")
    return None

def disconnect(client_ip):

    if not is_user_in_lab(client_ip):  # already disconnected
        print(f"User {client_ip} to disconnect is not in lab.")
        return "OK"

    url = f"api/user/{client_ip}/disconnect"
    method='PUT'
    try:
        # print(f"{method} {SRL_SERVICE}/{url}")
        response = requests.request(method, f"{SRL_SERVICE}/{url}", timeout=30)
        if response.status_code == 200:
            return response.json()
        t = time.time()
        dt = datetime.datetime.fromtimestamp(t)
        ts = dt.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] ERROR API Error fetching {url}: Status {response.status_code}")
    except requests.exceptions.RequestException as e:
        t = time.time()
        dt = datetime.datetime.fromtimestamp(t)
        ts = dt.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] ERROR Request failed for {url}: {e}")
    return None


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

def gz_get_paused():
    cmd = "gz topic -t /stats -e -n 1 > /tmp/_gz_stats"
    os.system(cmd)
    with open("/tmp/_gz_stats", "r") as f:
        for l in f.readlines():
            if "pause" in l and "true" in l:
                return True
    return False

# global
last_timestamp = time.time()
clients = {}
first = False

def tcp_listen():
    global last_timestamp, first
    #filter_expr = f"src host {lsb_ip} and src port {lsb_port} and dst host {client_ip}"
    filter_expr = f"src host {lsb_ip} and src port {lsb_port}"
    p = sub.Popen(('sudo', 'tcpdump', '-i', wg_if, '-n', '-l', filter_expr),
            stdout=sub.PIPE)
    for l in iter(p.stdout.readline, b''):
        l = l.strip().decode('utf-8')
#        c = f"{lsb_ip}.{lsb_port} > {client_ip}"
        if l:
            first = True
            last_timestamp = time.time()
            # print(f"{l[0:70]} ...")
            v = l.split(' ')
            vv = v[4].split('.')
            client_ip = vv[0]+'.'+vv[1]+'.'+vv[2]+'.'+vv[3]
            #print(client_ip)
            clients[client_ip] = last_timestamp

th = threading.Thread(target=tcp_listen, daemon=True)  # exit with main
th.start()


try:

    while True:
        time.sleep(check_interval) # every 30 sec.
        t = time.time()
        dt = datetime.datetime.fromtimestamp(t)
        ts = dt.strftime("%Y-%m-%d %H:%M:%S")

        # check users in lab
        users_in_lab = inlab()
        user_bookings = active_bookings()
        nc = 0
        nl = 0
        nb = 0
        try:
            nc = len(clients.keys())
            if users_in_lab is not None:
                nl = len(users_in_lab)
            if user_bookings is not None:
                nb = len(user_bookings)
            if nc>0:
                print(f"[{ts}] nr. clients: {nc}")
            if nc != nl or nc != nb:
                print(f"[{ts}] WARNING - clients: {nc}, inlab: {nl}, bookings: {nb}");
        except Exception as e:
            print(f"DEBUG: {e}")


        # update timestamp of users in lab
        if users_in_lab is not None:
            for user in users_in_lab:
                uid = user['id']
                client_ip = user['vpn_ip']
                #print(f"User in lab {uid} {client_ip}")
                if client_ip not in clients.keys():
                    clients[client_ip] = time.time()
                    print(f"[{ts}] New user in lab {uid} {client_ip}")


        # create list of users to be disconnected
        to_disconnect = []
        for c,lts in clients.items():
            d = t - lts
            actual_timeout_disconnect = timeout_disconnect if nl>0 else timeout_disconnect/2
            print(f"[{ts}] Check {c} : {d:.2f} < {actual_timeout_disconnect}")
            if (d>actual_timeout_disconnect):
                to_disconnect.append(c)

        # disconnect users
        for c in to_disconnect:

            # check if user is still in lab
            uinlab = is_user_in_lab(c)

            if not uinlab:
                print(" -- User not in lab, delete from the monitor list.")
                del clients[c]
                print(f"[{ts}] Client {c} disconnected.")
            else:   # disconnect user that is in lab
                r = disconnect(c)
                if r is None:
                    print(f"[{ts}] Disconnect {c} failed.")
                    print(" -- User still in lab, trying to disconnect later.")
                else:
                    del clients[c]
                    print(f"[{ts}] Client {c} disconnected.")


        # LSB 11 specific policy to start/stop simulation and GUI

        paused = gz_get_paused()
        if nc==0 and not paused:
            print("Simulation paused, GUI off")
            gz_pause(True)   # pause simulation
            gz_gui(False)    # kill gz gui
        elif nc>0 and paused:
            print("Simulation running, GUI on")
            gz_pause(False)  # unpause simulation
            gz_gui(True)     # start gz gui



        sys.stdout.flush()



except KeyboardInterrupt:
    print("\nProgram interrupted by user (Ctrl+C). Exiting.")
    # The daemon thread will automatically exit
except Exception as e:
    print(f"\nAn error occurred in the main loop: {e}")



