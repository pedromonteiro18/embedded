import json
import network
from umqtt.simple import MQTTClient
import machine
import time
from machine import Pin,I2C,RTC
import manipulatingdata
import formattime

THRESHOLD = 0.05
N_LARGEST_VALUES = 3
CTRL_REG1 = 0x20 #Control the sampling rate and take the board off low power mode.
LIS3DH_REG_CTRL4 = 0x23
OUT_X_L = 0x28 #LSB of X-Axis
OUT_X_H = 0x29 #MSB of X-Axis
OUT_Y_L = 0x2A #LSB of Y-Axis
OUT_Y_H = 0x2B #MSB of Y-Axis
OUT_Z_L = 0x2C #LSB of Z-Axis
OUT_Z_H = 0x2D #MSB of Z-Axis
DIVIDER = 16380
FIFTY_HZ_SAMPLING = 71
THRESHOLD_LDR = 2

#-----------------------------For connectivity and time-------------------------
class get_current_time():
    def __init__(self):
        self.time = 0
    def update_time(self,msg):
        self.time = msg

def sub_cb(topic,msg):
    time_from_broker = str(msg,'utf-8')
    currenttime.update_time(time_from_broker)

def creatingPayload(number_of_values, nLargestValues, nLargestValues_time, xyz):
    addG = 'g '.join(map(str,nLargestValues)) + "g"
    acc_data = ["%d max %s impact values: " %(number_of_values,xyz) + addG, nLargestValues_time]
    payload = json.dumps(acc_data)
    return payload

def connectToWifi(sta_if):

    sta_if.active(True)
    sta_if.connect('EEERover','exhibition')
    while(not sta_if.isconnected()):
        pass
    time.sleep(0.5)

#-----------------------------Connect and get time------------------------------
#Setting up the LED for control
led = machine.Pin(16,machine.Pin.OUT)

#Connect to the EEERover
sta_if = network.WLAN(network.STA_IF)
connectToWifi(sta_if)

#Connecting to the MQTT Broker for getting the time
client = MQTTClient('unnamed1','192.168.0.10')
client.connect()
currenttime = get_current_time()
client.set_callback(sub_cb)
client.subscribe(b'esys/time')
client.wait_msg()
client.check_msg()

time_right_now = json.loads(currenttime.time)["date"] #Gets the RFC string of the time at that instant

#Turn LED off when microcontroller has gotten time
if(len(time_right_now) > 0):
    led.low()

#Place time in the appropriate tuple for Machine.RTC
year = int(time_right_now[0:4])
month = int(time_right_now[5:7])
day = int(time_right_now[8:10])
hour = int(time_right_now[11:13])
minute = int(time_right_now[14:16])
second = int(time_right_now[17:19])
weekday = 4

rtc = machine.RTC()
rtc.datetime((year, month, day, weekday, hour, minute, second, 0))

#----------------------Once time has been received, start sensing---------------
sta_if.active(False)

#Find out which sensor is connected to the NodeMCU
i2c = I2C(scl = Pin(5),sda = Pin(4),freq = 500000)
addr_list = i2c.scan() #for switch control.

#Start sensing when there is an address
while(len(addr_list) == 0):
    addr_list = i2c.scan()
    if(len(addr_list) > 0):
       break
    time.sleep(1)

addr = addr_list[0] #Will assign an addr if a sensor is connected

i2c.writeto_mem(addr, CTRL_REG1, bytearray([FIFTY_HZ_SAMPLING])) #Set sampling rate

#Collecting data from the sensor
nLargestValues = [[] for q in range(3)]
nLargestValues_time = [[] for w in range(3)]
countXYZ = [0, 0, 0]

open_times = []
adc = machine.ADC(0)

#-----------------Record N largest impact values in every direction-------------
while(len(addr_list) > 0):

    firstValues = [manipulatingdata.normalise(i2c.readfrom_mem(addr,OUT_X_L,1), i2c.readfrom_mem(addr,OUT_X_H,1), DIVIDER),
                   manipulatingdata.normalise(i2c.readfrom_mem(addr,OUT_Y_L,1), i2c.readfrom_mem(addr,OUT_Y_H,1), DIVIDER),
                   manipulatingdata.normalise(i2c.readfrom_mem(addr,OUT_Z_L,1), i2c.readfrom_mem(addr,OUT_Z_H,1), DIVIDER)]
    secondValues = [manipulatingdata.normalise(i2c.readfrom_mem(addr,OUT_X_L,1), i2c.readfrom_mem(addr,OUT_X_H,1), DIVIDER),
                   manipulatingdata.normalise(i2c.readfrom_mem(addr,OUT_Y_L,1), i2c.readfrom_mem(addr,OUT_Y_H,1), DIVIDER),
                   manipulatingdata.normalise(i2c.readfrom_mem(addr,OUT_Z_L,1), i2c.readfrom_mem(addr,OUT_Z_H,1), DIVIDER)]

    for f in range(0, 3):
        diff = abs(secondValues[f] - firstValues[f])
        if(diff > THRESHOLD):
            if(countXYZ[f] > N_LARGEST_VALUES - 1):
                index = manipulatingdata.minimum(nLargestValues[f]) 
                if(diff > nLargestValues[f][index]):
                    nLargestValues[f][index] = diff
                    nLargestValues_time[f][index] = rtc.datetime()
            else:
                nLargestValues[f].append(diff)
                nLargestValues_time[f].append(rtc.datetime())
                countXYZ[f] += 1

    #Check to see if suitcase is open
    if(adc.read()*(3.3/1023) > THRESHOLD_LDR):
        if(len(open_times) < 1):
            open_times.append(rtc.datetime())

    time.sleep(0.05)
    addr_list = i2c.scan()

#-----------------------Sending the data to the MQTT Broker---------------------
connectToWifi(sta_if)
client = MQTTClient('unnamed1','192.168.0.10')
client.connect()
xyz = ["x", "y", "z"]
for i in range(0, 3):

    nLargestValues_time[i] = formattime.convert_time(nLargestValues_time[i])
    payload = creatingPayload(N_LARGEST_VALUES, nLargestValues[i], nLargestValues_time[i], xyz[i])
    client.publish('/unnamed1/test', bytes(payload, 'utf-8'))

if(len(open_times) > 0):
    open_times = "Your suitcase was opened at: " + str(formattime.convert_time(open_times))
    payload_times = json.dumps(open_times)
    client.publish('/unnamed1/test', bytes(payload_times, 'utf-8'))
else:
    client.publish('/unnamed1/test', bytes("Your suitcase was never opened.", 'utf-8'))

#Set LED high to indicate that data has been sent
led.high()
