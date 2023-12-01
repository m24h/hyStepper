import serial
import signal
import threading
import sys

if len(sys.argv)<3:
	print("Usage: "+sys.argv[0]+" <COM?> <foc/trk> <1:900RPM 2:450RPM 3:230RPM>")
	sys.exit(0)

def quit(signum, frame):
	sys.exit(0)

run=True

def work():
	try:
		ser = serial.Serial("\\\\.\\"+sys.argv[1], 250000, timeout=0.5)
		if sys.argv[2]=="foc":
			ser.write(b'\xFD\x00\x03\x00\x01\x11')
		else:
			ser.write(b'\xFD\x00\x03\x00\x01\x10')
		if sys.argv[3]=="3":
			b=b'\x01\x00\x00\x00'*2000000
		elif sys.argv[3]=="2":
			b=b'\x01\x00'*4000000
		else:
			b=b'\x01'*8000000
		ser.write(b'\xF8')
		ser.write(b)
	except Exception as e:
		print("Exception：", e)
	global run
	run=False

signal.signal(signal.SIGINT, quit)
signal.signal(signal.SIGTERM, quit)
work_thread = threading.Thread(target=work)
work_thread.daemon = True
work_thread.start()
while run:
	pass
    