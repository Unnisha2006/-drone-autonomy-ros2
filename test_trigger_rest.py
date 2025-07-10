from flask import Flask, request
import requests

app = Flask(__name__)

@app.route('/trigger', methods=['POST'])
def trigger():
    command = request.json.get('command')
    if command:
        res = requests.post("http://localhost:8000/ai_trigger", json={"data": command})
        return f"Command sent: {command}", 200
    return "Missing command", 400

if __name__ == '__main__':
    app.run(port=5000)
