from flask import Flask, request, jsonify
import paho.mqtt.client as mqtt
from paho.mqtt import client as mqtt_client
app = Flask(__name__)

# MQTT configuration
MQTT_BROKER = "localhost"  # The default gateway IP for Docker containers on Linux
 

MQTT_PORT = 1883
MQTT_TOPIC_COMMAND = "robot/command"

client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, 60)
@app.route('/')
def home():
    return jsonify({"status": "App is running"})

@app.route('/health')
def health_check():
    return {"status": "healthy"}, 200
@app.route('/send_command', methods=['POST'])
def send_command():
    data = request.json
    command = data.get("command")
    if not command:
        return jsonify({"error": "Command not provided"}), 400
    
    mqtt_client.publish(MQTT_TOPIC_COMMAND, command)
    return jsonify({"status": "Command sent", "command": command})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
