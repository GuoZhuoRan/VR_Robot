import datetime
import logging
import random
import time
from flask import Flask, render_template, request, jsonify


app = Flask(__name__)

button_enabled = True

log = logging.getLogger('werkzeug')



@app.route('/start-server', methods=['POST'])
def start_server():
    streamer_dev.running = True # type: ignore
    listener_thread = threading.Thread(target=streamer_dev.start_vision_pro_server)
    listener_thread.start()
    return jsonify({"status": "server started"})

@app.route('/')
def button2():
    return render_template('button2.html')


@app.route('/get_button_enabled', methods=['GET'])
def get_button_enabled():
    global button_enabled
    res=button_enabled
    button_enabled=False
    return jsonify(enabled=res)



if __name__ == '__main__':
    button_enabled=True
    # 绑定到0.0.0.0，使得局域网内其他设备也可以访问
    app.run(host='0.0.0.0', port=5000)
