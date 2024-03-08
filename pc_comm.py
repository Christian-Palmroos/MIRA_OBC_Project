import serial
import time
import os

found = False
i = 0
while not found:
    if i > 10:
        raise Exception("Device not found under /dev/ttyACMx!")
    try:
        ser = serial.Serial(f'/dev/ttyACM{i}', 115200, timeout = 2, write_timeout = 5)
        print(f'Device found at /dev/ttyACM{i}')
        found = True
    except:
        print(f'Was not /dev/ttyACM{i}')
        i+=1
        

def main():

    echo_switch = False
    # connect = ser.readline().strip()
    # if connect == 'CONNECTED':
    #   print("OBC connected!")
    # else:
    #   print("Failed to connect to the OBC.")
    #   raise Exception("Check connection to the OBC.")
    start = time.time()
    print("Emptying queue...")
    while time.time()-start < 5:
        print(ser.readline())

    while True:
        print()
        userinput = input("Input (type h for help): ")

        if userinput == "h":
            print("Command options:")
            print("     flightmode - OBC goes to flightmode and terminates USB communication.")
            print("     ping - Pings the OBC to see if it replies.")

        elif userinput == "flightmode":
            print("Going into flight mode...")
            
            msg = f"\x00\x09".encode()
            
            print(f"Waiting for response...")
            ser.write(msg)
            
            if echo_switch:
                echo = ser.readline()
                print("Echo:", echo, "| Decoded:", echo.decode('latin-1'))
                
            response = ser.readline().decode('latin-1')
            print("Response:", response)
            
            if response.startswith("OK"):
                break
            
        elif userinput == "ping":
            print("PING")
            
            msg = f"\x00\x08".encode()
            
            print(f"Waiting for response...")
            ser.write(msg)
            
            if echo_switch:
                echo = ser.readline()
                print("Echo:", echo, "| Decoded:", echo.decode('latin-1'))
                
            response = ser.readline().decode('latin-1')
            print("Response:", response)
        
        else:
            print("Not a command.")
 
if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        ser.close()
        print("\nKilled.")
