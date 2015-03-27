import serial_utils

from myserial import ser

ser.flushInput()
ser.flushOutput()

serial_utils.WriteByte(ser, 5)#ask for thetas

theta0 = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
theta1 = serial_utils.Read_Two_Bytes_Twos_Comp(ser)
nl_check = serial_utils.Read_Byte(ser)
assert nl_check == 10, "newline problem"

print('theta0 = %i' % theta0)
print('theta1 = %i' % theta1)

