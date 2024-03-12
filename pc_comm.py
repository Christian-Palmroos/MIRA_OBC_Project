import serial
import time
import os

print("===============")
print("OBC setup script")
print("===============")
print()

# Toggles sent message echo from OBC
echo_switch = False

# Find the Virtual Com Port the device is under
found = False
i = 0
while not found:
    if i > 20:
        raise Exception("Device not found under /dev/ttyACMx!")
    try:
        ser = serial.Serial(f'/dev/ttyACM{i%10}', 115200, timeout = 2, write_timeout = 5)
        print(f'Device found at /dev/ttyACM{i%10}')
        found = True
    except:
        print(f'Was not /dev/ttyACM{i%10}')
        i+=1
        

def main():

    start = time.time()
    print("Emptying queue...")
    while time.time()-start < 10:
        print(ser.readline().decode('latin-1'), end = "")

    while True:
        
        print()
        userinput = input("Input (type h for help): ")


        if userinput == "h":
            print("Command options:")
            print("     flightmode - OBC goes to flightmode and terminates USB communication.")
            print("     ping - Pings the OBC to see if it replies.")
            print("     readmode - Swap to USB line read-only mode.")
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
            
            
        elif userinput == "readmode":
            print("OK, reading...")
            while True:
                print(ser.readline().decode('latin-1'), end = "")
        
        
        else:
            print("Not a command.")
            
    
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
