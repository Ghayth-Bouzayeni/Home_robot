from flask import Flask, request, jsonify
import paho.mqtt.client as mqtt

app = Flask(__name__)

# MQTT configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_COMMAND = "robot/command"

mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

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
