import datetime
import logging
import random
import time
from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

# 全局变量来保存按钮的启用状态
button_enabled = True

log = logging.getLogger('werkzeug')
# log.setLevel(logging.ERROR)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/execute_function', methods=['POST'])
def execute_function():
    # 这里放置你想要执行的Python函数
    result = my_function()
    return jsonify({'result': result})

@app.route('/execute_function2', methods=['POST'])
def execute_function2():
    # 这里放置你想要执行的Python函数
    result = my_function()
    return jsonify({'result': result})

def my_function():
    # 这是一个示例函数
    return "Function executed successfully! 1111111"

@app.route('/get_number', methods=['GET'])
def get_number():
    # 生成一个随机数，作为示例。你可以在这里替换为其他逻辑
    number  = datetime.datetime.now()
    return jsonify({'number': number})

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
