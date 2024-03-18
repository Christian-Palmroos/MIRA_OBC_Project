import serial
import glob
import time
import os

print("===============")
print("OBC setup script")
print("===============")
print()

# Toggles sent message echo from OBC
echo_switch = False

# Find the Virtual Com Port the device is under
def find_ports():
    ports = glob.glob('/dev/ttyACM[0-9]*')

    res = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            res.append(port)
        except:
            pass
    return res

setup = False
while not setup:
    print(find_ports())
    userinput = input("Which of these devices would you like to connect to (answer with index of ACM only, e.g., 0)?    ")
    try:
        ser = serial.Serial(f'/dev/ttyACM{userinput}', 115200, timeout = 2, write_timeout = 5)
        setup = True
    except:
        print("Wrong input!")


def main():

    start = time.time()
    print("Emptying queue...")
    while time.time()-start < 3:
        print(ser.readline().decode('latin-1'), end = "")

    while True:
        
        print()
        userinput = input("Input (type h for help): ")


        if userinput == "h":
            print("Command options:")
            print("     flightmode - OBC goes to flightmode and terminates USB communication.")
            print("     ping - Pings the OBC to see if it replies.")
            print("     testlora - Tests LORA data sending.")


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
            
        
        elif userinput == "testlora":
            print("Sending test packet with LORA...")
            
            msg = f"\x02\x02".encode()
            
            print(f"Waiting for response...")
            ser.write(msg)
            
            if echo_switch:
                echo = ser.readline()
                print("Echo:", echo, "| Decoded:", echo.decode('latin-1'))
                
            response = ser.readline().decode('latin-1')
            print("Response:", response)
        
        
        else:
            print("Not a command.")
            
    # Read mode
    while True:
        print(ser.read().decode('latin-1'), end = "")
            
    
if __name__ == "__main__":
    
    try: 
        main()
        
    except KeyboardInterrupt:
        ser.close()
        print("============")
        print("\nKilled.")
        print("============")
    
    except:
        ser.close()
