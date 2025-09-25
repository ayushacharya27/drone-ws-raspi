# app.py
import eventlet
eventlet.monkey_patch()
import json
from flask import Flask, render_template
from flask_socketio import SocketIO

# Define the path to the file we will be reading
FILE_PATH = r"/tmp/drone_data.json"

app = Flask(__name__)
socketio = SocketIO(app, async_mode='eventlet', cors_allowed_origins="*")

def data_emitter_thread():
    """Periodically reads the JSON file and sends data to clients."""
    while True:
        try:
            with open(FILE_PATH, 'r') as f:
                data = json.load(f)
            socketio.emit('drone_data', json.dumps(data))
        except FileNotFoundError:
            print(f"Waiting for file to be created at {FILE_PATH}...")
        except json.JSONDecodeError:
            print(f"Error decoding JSON from {FILE_PATH}. File might be corrupt or being written.")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            
        socketio.sleep(0.1) # Read and send at 10Hz

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print('Client connected')

if __name__ == '__main__':
    # Start the background thread for reading the file and sending data
    eventlet.spawn(data_emitter_thread)
    
    # Start the Flask-SocketIO server
    print("Starting Flask server to read from JSON file...")
    socketio.run(app, host='0.0.0.0', port=8000)