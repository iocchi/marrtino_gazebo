import asyncio
import websockets
import json

from control import MARRtinoController

robot = MARRtinoController()

async def echo(websocket):
    print(f"Client connected from {websocket.remote_address}")
    try:
        async for message in websocket:
            print(f"Received message from client: {message}")
            try:
                data = json.loads(message)
                if "text" in data:
                    received_text = data["text"]
                    print(f"Text:\n{received_text}")
                    await websocket.send(json.dumps({"status": "success", "message": "Text received!"}))
                if "code" in data:
                    received_code = data["code"]
                    print(f"Python code:\n{received_code}")
                    await websocket.send(json.dumps({"status": "success", "message": "Code running!", "disable_send": "true"}))
                    try:
                        exec(received_code)
                        await websocket.send(json.dumps({"status": "success", "message": "Code completed!", "disable_send": "false"}))
                    except Exception as e:
                        print(f"Error: {e}")
                        await websocket.send(json.dumps({"status": "error", "message": f"{e}", "disable_send": "false"}))
                else:
                    print("Received message does not contain 'text' key.")
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

async def main():
    # Start the WebSocket server on localhost, port 8765
    async with websockets.serve(echo, "localhost", 8765) as server:
        print("WebSocket server started on ws://localhost:8765")
        await server.serve_forever()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())

