import time
import serial

class Com_serial():
    def __init__(self,port,baudrate=9600):
        self.port=port
        self.baudrate=baudrate
        self.device=None

    def start(self): 
        self.device=serial.Serial()
        self.device.baudrate = self.baudrate
        self.device.port = self.port
        self.device.open()
        time.sleep(2)
        
    def send(self,data):
        self.device.write(data.encode('utf-8'))
    

