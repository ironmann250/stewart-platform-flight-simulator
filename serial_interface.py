import time
import serial

class Com_serial():
    def __init__(self,port,baudrate=9600,timeout=0.05):
        self.port=port
        self.baudrate=baudrate
        self.device=None
        self.timeout=timeout=timeout

    def start(self): 
        self.device=serial.Serial()
        self.device.baudrate = self.baudrate
        self.device.port = self.port
        self.device.timeout = self.timeout
        self.device.open()
        time.sleep(2)
        
    def send(self,data):
        self.device.write(data.encode('utf-8'))

    def read_position_blocking(self):
        ret=self.device.readline()
        while ret in [b"",None]:
            ret=self.device.readline()
        ret=ret.decode('utf-8').replace('\r\n','')
        return ret.split("/")
    

