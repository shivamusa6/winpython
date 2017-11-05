import try3
from flask import Flask, jsonify

app = Flask(__name__)

tasks =try3.main()
data=try3.create_data_array()
location=data[0]
demands=data[1]
print(location)
print(demands)


@app.route('/', methods=['GET'])
def get_tasks():
    return  tasks

if __name__ == '__main__':
    app.run(debug=True)

