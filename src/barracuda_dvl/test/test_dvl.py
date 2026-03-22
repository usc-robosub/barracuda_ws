import socket
import json
import time

# Define server address and port
TCP_IP = "0.0.0.0"  # Listen on all available interfaces
TCP_PORT = 16171

# DVL data to be sent
DVL_DATA = {
    "time": 106.3935775756836,
    "vx": -3.713480691658333e-05,
    "vy": 5.703703573090024e-05,
    "vz": 2.4990416932269e-05,
    "fom": 0.00016016385052353144,
    "covariance": [
        [2.4471841442164077e-08, -3.3937477272871774e-09, -1.6659699175747278e-09],
        [-3.3937477272871774e-09, 1.4654466085062268e-08, 4.0409570134514183e-10],
        [-1.6659699175747278e-09, 4.0409570134514183e-10, 1.5971971523143225e-09],
    ],
    "altitude": 0.4949815273284912,
    "transducers": [
        {
            "id": 0,
            "velocity": 0.00010825289791682735,
            "distance": 0.5568000078201294,
            "rssi": -30.494251251220703,
            "nsd": -88.73271179199219,
            "beam_valid": True,
        },
        {
            "id": 1,
            "velocity": -1.4719001228513662e-05,
            "distance": 0.5663999915122986,
            "rssi": -31.095735549926758,
            "nsd": -89.5116958618164,
            "beam_valid": True,
        },
        {
            "id": 2,
            "velocity": 2.7863150535267778e-05,
            "distance": 0.537600040435791,
            "rssi": -27.180519104003906,
            "nsd": -96.98075103759766,
            "beam_valid": True,
        },
        {
            "id": 3,
            "velocity": 1.9419496311456896e-05,
            "distance": 0.5472000241279602,
            "rssi": -28.006759643554688,
            "nsd": -88.32147216796875,
            "beam_valid": True,
        },
    ],
    "velocity_valid": True,
    "status": 0,
    "format": "json_v3.1",
    "type": "velocity",
    "time_of_validity": 1638191471563017,
    "time_of_transmission": 1638191471752336,
}

# Convert the dictionary to a JSON string
JSON_DATA = json.dumps(DVL_DATA) + "\n"  # Newline to signal end of message


def start_server():
    """Starts a TCP server that sends DVL data to a client."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((TCP_IP, TCP_PORT))
        server_socket.listen(1)  # Allow only one client connection

        print(f"Server listening on {TCP_IP}:{TCP_PORT}...")

        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Client {client_address} connected.")

            try:
                while True:
                    client_socket.sendall(JSON_DATA.encode("utf-8"))  # Send JSON data
                    print(f"Sent DVL Data to {client_address}")

                    time.sleep(1)  # Simulate real-time data sending

            except (BrokenPipeError, ConnectionResetError):
                print(f"Client {client_address} disconnected.")
            finally:
                client_socket.close()


if __name__ == "__main__":
    start_server()

