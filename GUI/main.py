from flask import Flask, request, render_template
import serial
from serial.tools import list_ports

app = Flask(__name__)

# Initialize an empty list for available ports
available_ports = []

def update_available_ports():
    global available_ports
    available_ports = [port.device for port in list_ports.comports()]

# Update the available ports initially
update_available_ports()
ports = ["",""];
sers = ["",""];

@app.route('/')
def index():
    # Update the available ports every time the page is loaded
    update_available_ports()
    return render_template('index.html', available_ports=available_ports)

@app.route('/toggle_channel', methods=['POST'])
def toggle_channel():
    try:
        port = request.form.get('port')
        command = request.form.get('command')
        if port == ports[0]:
            ser = sers[0]
        elif port == ports[1]:
            ser = sers[1]
        else:
            print("Invalid port")
            return "Invalid port"
        print(command)
        ser.read_all()  # Read one line (you can also use ser.read() for binary data)
        ser.write(command.encode())
        print("Command sent:", command);
        response = "Invalid";
        trycount = 0;

        while trycount <= 50: #sometimes takes up to 5 lines to read
            trycount = trycount+1;
            print(trycount)
            try:
                response = ser.readline().decode()  # Read up to 50 characters from the serial port
                print("Response received:", response)
            except:
                print("Response invalid")
            print("Expected: Channel " + command[0] + " turned ON");
            if "Channel " + command[0] + " turned ON" in response:
                print("ON")
                return "ON"
            elif f"Channel " + command[0] + " turned OFF" in response:
                print("OFF")
                return "OFF"
        return "no valid response"
    except Exception as e:
        # Handle serial communication error
        print("Serial communication error:", str(e))
        return "Serial error"

@app.route('/connect_serial', methods=['POST'])
def connect_serial():
    port1_ = request.form.get('port1')
    port2_ = request.form.get('port2')
    print("POSTport1:", port1_)
    print("POSTport2:", port2_)
    if (port1_ != None):
        ports[0] = port1_
        sers[0] = serial.Serial(ports[0], baudrate=921600)
        print("Connected:", ports[0])
        return "Serial connection established"
    if (port2_ != None):
        ports[1] = port2_
        sers[1] = serial.Serial(ports[1], baudrate=921600)
        print("Connected:", ports[1])
        return "Serial connection established"
    # Implement your serial connection setup here

if __name__ == '__main__':
    # Initialize serial ports
    app.run(debug=True)
