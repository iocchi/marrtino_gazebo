import asyncio
import websocket
import json
import time

#wshost = "ws://localhost:9876/"
#wshost = "ws://172.19.0.1:3080/code_ws"
wshost = "ws://localhost:3080/code_ws"

#wshost = "ws://10.95.1.5:3080/code_ws"

class WSRobot():

    def __init__(self):
        self.websocket = websocket.create_connection(wshost)        
        print(f"Connected to {wshost}")

    def __del__(self):
        self.websocket.close()

    def send(self, robot_cmd):
    
        message = {
            "robotfn": robot_cmd+"\n",
        }
        
        self.websocket.send(json.dumps(message))

        response = self.websocket.recv()
        print(f"Risposta dal server: {json.loads(response)}")

        response = self.websocket.recv()
        jr = json.loads(response)
        print(f"Risposta dal server: {jr}")

        return jr['result']


class Robot:
    def __init__(self):
        self.ws_robot = WSRobot()

    def __del__(self):
        pass

    def __getattr__(self, name):
        """
        Viene chiamato quando si accede a un attributo che non esiste.
        Restituisce una funzione che cattura gli argomenti.
        """
        def wrapper(*args, **kwargs):

            # Trasformiamo args (posizionali) in stringhe: 1, 2, 3
            pos_args_str = ", ".join(map(repr, args))
            
            # Trasformiamo kwargs (nominali) in stringhe: chiave=valore
            kw_args_str = ", ".join([f"{k}={repr(v)}" for k, v in kwargs.items()])
            
            # Uniamo le due stringhe per rappresentare la chiamata completa
            all_params = ", ".join(filter(None, [pos_args_str, kw_args_str]))
            full_call_str = f"robot.{name}({all_params})"
                        
            print(f"[Robot Proxy] to send: {full_call_str}")
            
            # Se abbiamo un client websocket, inviamo il comando
            r = None
            if self.ws_robot:
                r = self.ws_robot.send(full_call_str)
            #    print(f"[Robot Proxy] Invio JSON al WebSocket: {json.dumps(command)}")
            
            return r

        return wrapper


if __name__ == "__main__":

    # --- Esempio di utilizzo ---

    # Simuliamo un oggetto Robot
    robot = Robot()

    # Possiamo chiamare QUALSIASI metodo, anche se non definito!
    robot.left(90)
    robot.right(90)
    robot.get_image()

    time.sleep(3)

    print(f"\nDone\n")


