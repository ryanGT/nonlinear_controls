import serial_utils
import sys

#righthand side for me
#portname = '/dev/tty.usbmodem1411'
#lefthand side for me
#portname = '/dev/tty.usbmodem1421'

#Raspberry Pi USB serial port:
portname = '/dev/ttyACM0'
if sys.platform == 'darwin':
    #max osx
    portname = '/dev/tty.usbmodem1411'
    
ser = serial_utils.Open_Serial(portname)
ser.flushInput()
ser.flushOutput()

debug_line = serial_utils.Read_Line(ser)
line_str = ''.join(debug_line)

print(line_str)
