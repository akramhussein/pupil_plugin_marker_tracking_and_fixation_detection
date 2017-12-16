import serial

class Serial():
    """handle serial port communication"""
    def __init__(self, port, baudrate):
        self.ser = serial.Serial()
        self.ser.baudrate = baudrate
        self.ser.port = port
        logging.info('Opened serial port ' + str(self.ser))
        self.ser.open()

    def write(self, data):
        if self.ser.is_open:
            self.ser.write(data)

    def close(self):
        self.ser.close()
