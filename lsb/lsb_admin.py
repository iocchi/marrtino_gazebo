import json
import requests
import os, sys

import argparse

import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web

if os.getenv("WGIF") != 'lsb11':
    sys.exit(0)


# --- Configurazione --- (valori di default - impostati da linea di comando)

SRL_SERVICE_HOST = '10.96.0.2'
SRL_SERVICE_PORT = 5000
WEBSOCKET_PORT = 9877


# --- Gestione API SRL (Sincrona con Requests) ---

def safe_fetch_json(url, method='GET'):
    """Esegue una richiesta HTTP sincrona al servizio SRL."""
    try:
        response = requests.request(method, f"{SRL_SERVICE_URL}{url}", timeout=5)
        if response.status_code == 200:
            return response.json()
        print(f"API Error fetching {url}: Status {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Request failed for {url}: {e}")
    return None


# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def open(self):

        self.clientIP = self.request.headers.get('X-Real-Ip', '')
        if self.clientIP=='':
            self.clientIP = self.request.remote_ip
        if self.clientIP=='::1':  # localhost
            self.clientIP = 'localhost'

        print(f"Client connesso: {self.clientIP}")

    def check_origin(self, origin):
        print("-- Request from %s" %(origin))
        return True

    def on_message(self, message):

        try:
            data = json.loads(message)
            action = data.get('action')

            # Requisito 1: Richiesta di aggiornamento di tutti i dati statici
            if action == 'fetch_all_data':
                print("Ricevuta richiesta di fetch_all_data.")
                context = self.get_all_srl_data()
                # Invia tutti i dati al client in un unico payload
                self.write_message(json.dumps({
                    'type': 'full_update',
                    'context': context
                }))

            # Requisito 2: Richiesta di azioni (Disconnect, Set Available/Unavailable)
            elif action in ['disconnect', 'set_available', 'set_unavailable']:
                handle_user_action(websocket, action, vpn_ip)
            else:
                self.write_message(json.dumps({'type': 'error', 'message': f'Azione non riconosciuta: {action}'}))

        except Exception as e:
            print(f"ERROR in WebSocket handler: {e}")


    def on_close(self):
        print("Connection closed from %s" %(self.clientIP))

    def on_ping(self, data):
        print("Ping received: %s" %(data))

    def on_pong(self, data):
        print("Pong received: %s" %(data))


    def get_all_srl_data(self):
        """Recupera tutti i dati statici dal servizio SRL."""
        context = {}

        context['ip'] = self.clientIP
        context['user'] = None  # safe_fetch_json("/api/user/by-ip/{self.clientIP}")
        context['inlab'] = safe_fetch_json("/api/service/inlab")
        context['waiting'] = safe_fetch_json("/api/service/waiting")
        context['bookings'] = safe_fetch_json("/api/service/bookings")

        return context

    def handle_user_action(self, action):
        """Esegue azioni utente (disconnessione, disponibilità lab) tramite API."""
        url = ""
        method = ""

        if action == 'disconnect':
            url = f"/api/user/{self.clientIP}/disconnect"
            method = 'PUT'
        elif action == 'set_available':
            url = "/api/service/availability/true"
            method = 'PATCH'
        elif action == 'set_unavailable':
            url = "/api/service/availability/false"
            method = 'PATCH'

        if url:
            result = safe_fetch_json(url, method)
            if result is not None:
                self.write_message(json.dumps({'type': 'action_result', 'action': action, 'status': 'success'}))
            else:
                self.write_message(json.dumps({'type': 'action_result', 'action': action, 'status': 'error', 'message': 'API call failed.'}))
        else:
            self.write_message(json.dumps({'type': 'action_result', 'action': action, 'status': 'error', 'message': 'Invalid action URL.'}))





if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='LSB Service')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Host to bind the ws service (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=WEBSOCKET_PORT, help='Port to bind the ws service')
    parser.add_argument('--srl-host', type=str, default=SRL_SERVICE_HOST, help='Host of the SRL service')
    parser.add_argument('--srl-port', type=int, default=SRL_SERVICE_PORT, help='Port of the SRL service')
    args = parser.parse_args()

    WEBSOCKET_PORT = args.port
    SRL_SERVICE_URL = f"http://{args.srl_host}:{args.srl_port}"

    print(f"🚀 LSB ws running at http://0.0.0.0:{WEBSOCKET_PORT}")
    print(f"🔗 Connecting to SRL service at {SRL_SERVICE_URL}")
    #print(f"🌐 Browser will connect to SRL service at {SRL_SERVICE_URL_BROWSER}")

    # Start websocket server
    application = tornado.web.Application([(r'/lsbadmin', MyWebSocketServer),])
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(WEBSOCKET_PORT)

    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print("-- Keyboard interrupt --")

