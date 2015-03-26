import serial_utils

#righthand side for me
#portname = '/dev/tty.usbmodem1411'
#lefthand side for me
portname = '/dev/tty.usbmodem1421'

ser = serial_utils.Open_Serial(portname)
ser.flushInput()
ser.flushOutput()

debug_line = serial_utils.Read_Line(ser)
line_str = ''.join(debug_line)

print(line_str)
