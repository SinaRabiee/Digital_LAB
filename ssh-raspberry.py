import serial
import getch

serialport = serial.Serial("/dev/ttyS0")
serialport.baudrate = 115200

while True:
    x = getch.getch()
    if "W" == x.upper():
        # Forwards
        command = "+100+10015+00"
    elif "S" == x.upper():
        # Backwards
        command = "-250-25015+00"
    elif x == "A" or x == "a":
        # Left
        command = "-150+15015+00"
    elif x == "D" or x == "d":
        # Right
        command = "+150-15015+00"
    elif x == "h" or x == "H":
        # Stop
        command = "+000+00015+00"
    else:
        break

    serialport.write(command.encode())
