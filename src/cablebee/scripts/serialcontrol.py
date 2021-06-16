import serial
import serial.tools.list_ports
import time

class StepperController:
    ser = None

    def __init__(self):
        self.ser = self.connectSerial()

    def connectSerial(self):
        while True:
            print("Checking for open serial ports...")
            arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if "Maple"
                in p.description  # may need tweaking to match new arduinos
            ]
            all_ports = [p.device for p in serial.tools.list_ports.comports()]

            if not arduino_ports:
                if not all_ports:
                    print("No ports found, exiting")
                    quit()
                print("No default found, but we have these:")
                for i, port in enumerate(all_ports):
                    print(str(i) + ": " + port)
                sel = int(input("please choose number..."))
                motherboardport = all_ports[sel]
            else:
                motherboardport = arduino_ports[0]

            ser = serial.Serial(
                port=motherboardport,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1,
            )

            if ser.is_open:
                print("Serial Connected: ", motherboardport)
                return ser
            else:
                print("Please check your connection to the motherboard.")
                input("Press enter when you think it is fixed...")

    def sendCommand(self, input):
        out_command = bytes(input + '\n', 'ASCII')
        self.ser.write(out_command)

    def enableSteppers(self):
        self.sendCommand('M17')
        self.ser.readline()

    def disableSteppers(self):
        self.sendCommand('M18')
        self.ser.readline()

    def getCurrentPosition(self):
        command = 'M114'
        self.sendCommand(command)
        time.sleep(0.01)
        print(self.ser.readline())
        print(self.ser.readline())

    def absolutePositioning(self):
        self.sendCommand('G90')
        self.ser.readline()

    def relativePositioning(self):
        self.sendCommand('G91')
        self.ser.readline()

    def setPosition(self, x, y, z, e):
        command = 'G92 ' + 'X' + str(x) + ' Y' + \
            str(y) + ' Z' + str(z) + ' E' + str(e)
        self.sendCommand(command)
        self.ser.readline()

    def linearMove(self, x, y, z, e):
        command = 'G0 ' + 'X' + str(x) + ' Y' + str(y) + \
            ' Z' + str(z) + ' E' + str(e)
        self.sendCommand(command)
        self.ser.readline()
