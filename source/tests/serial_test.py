
'''
Reads incoming bytes from serial port,
prints them on screen and sends them back.
Based on the work of Daniel Jacoby.
'''

import serial
	
ser = serial.Serial(port='COM5',
                    baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=0,
                    rtscts=True)
print(ser.name)																	# check which port was really used

data = 0
while data != b'q':
	# print("Waiting for data")
	data = ser.read()															# read serial port
	if data:
		c = str(data, encoding='ascii')
		if c == 'R' or c == 'C' or c == 'O':
			print("\n")
		print(c)
		# ser.write((int.from_bytes(data, 'big') + 1).to_bytes(1, 'big'))		# write data back to the port
		# ser.write(data)														# write data back to the port

ser.close()																		# close port
