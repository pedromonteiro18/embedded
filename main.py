import smbus  # import sudo apt-get install python3-smbus i2c-tools
from time import sleep  # import sleep
import math
import json
import paho.mqtt.client as mqtt
import sys
import RPi.GPIO as GPIO
from twilio.rest import Client

account = "AC2650e854a75522a55556c046e458d3e3"
token = "5c316eabc51ae0d01886182a30633f55"
clientPhone = Client(account, token)

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

warning_sent = False

client = mqtt.Client()
client.tls_set(ca_certs="/home/pi/embedded/mosquitto.org.crt", certfile="/home/pi/embedded/client.crt", keyfile="/home/pi/embedded/client.key")
connected=False
while (not connected):
	try:
		client.connect("test.mosquitto.org", port=8884)
		connected=True
	except:
		print("connection failed trying again")
		
if client.connect("test.mosquitto.org", port=8884) == 0:
    print("Connection successful")
else:
    print("Error connection unsuccessful")
    print(mqtt.error_string(RETURN_CODE))
    sys.exit(1)
MSG_INFO = client.publish("IC.embedded/patriots/tmp", "Door Locked")
mqtt.error_string(MSG_INFO.rc)  # MSG_INFO is result of publish()


def on_message(client, userdata, message):
    message_string = str(message.payload)
    message_string = message_string[2:len(message_string)-1]
    if message_string == "Active":
        global warning_sent
        warning_sent = False
        print("Hello")
    elif message_string == "Not Active":
        global warning_sent
        warning_sent = True
        print("not Active")
    print("Received message:{} on topic {}".format(message.payload, message.topic))

# def on_message(client, userdata, message):
#     print("Received message '" + str(message.payload) + "' on topic '"
#     + message.topic + "' with QoS " + str(message.qos))


client.on_message = on_message
client.subscribe("IC.embedded/patriots/test")
client.loop()

# some MPU6050 Registers and their Address
Register_A = 0  # Address of Configuration register A
Register_B = 0x01  # Address of configuration register B
Register_mode = 0x02  # Address of mode register

X_axis_H = 0x03  # Address of X-axis MSB data register
Z_axis_H = 0x05  # Address of Z-axis MSB data register
Y_axis_H = 0x07  # Address of Y-axis MSB data register
declination = -0.00669  # define declination angle of location where measurement going to be done
pi = 3.14159265359  # define pi value


def Magnetometer_Init():
    # write to Configuration Register A
    bus.write_byte_data(Device_Address, Register_A, 0x70)

    # Write to Configuration Register B for gain
    bus.write_byte_data(Device_Address, Register_B, 0xa0)

    # Write to mode Register for selecting mode
    bus.write_byte_data(Device_Address, Register_mode, 0)


def read_raw_data(addr):
    # Read raw 16-bit value
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from module
    if value > 32768:
        value = value - 65536
    return value


bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x1e  # HMC5883L magnetometer device address

Magnetometer_Init()  # initialize HMC5883L magnetometer

print(" Reading Heading Angle")

old_heading = 0

while True:

    client.loop()
    
    # Read Accelerometer raw value
    x = read_raw_data(X_axis_H)
    z = read_raw_data(Z_axis_H)
    y = read_raw_data(Y_axis_H)

    heading = math.atan2(y, x) + declination

    # Due to declination check for >360 degree
    if heading > 2 * pi:
        heading = heading - 2 * pi

    # check for sign
    if heading < 0:
        heading = heading + 2 * pi

    # convert into angle
    heading_angle = int(heading * 180 / pi)

    if abs(heading_angle - old_heading) > 10 and old_heading != 0 and warning_sent == False:
        warning_sent = True
        MSG_INFO = client.publish("IC.embedded/patriots/tmp", "Door Opened")
        num = ['07767292464', '07452900152', '07508997292']
        for number in num:
            message = clientPhone.messages.create(to=number, from_="+447403922805", body="Door Opened!")
        mqtt.error_string(MSG_INFO.rc)  # MSG_INFO is result of publish()
        p = GPIO.PWM(18, 10000000000000)
        p.start(50)
        sleep(6)
        p.stop()
        GPIO.cleanup()

    print("Heading Angle = %d°" % heading_angle)
    
    old_heading = heading_angle

    sleep(1)

    payload = json.dumps({
        "Heading Angle ": heading_angle
    })






