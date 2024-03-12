import serial
import time
import os
from csv import writer

print("===============")
print("LORA Receiver script")
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
        
# Check if data file exists and create/open it      
data_path = "data.csv"
f_object = open(data_path, "x" if not os.path.exists(data_path) else "a")


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
            print("     flightmode - Receiver goes to flightmode and starts writing received LORA data to file.")
            print("     ping - Pings the receivier to see if it replies.")
            print("     testlora - Tests LORA data receiving.")


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
                # Check that the datafile exists for writing
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
            print("Receiving test packet with LORA...")
            
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
      
      
    
    while True:
        
        # Read incoming data
        datarow = ser.readline().decode('latin-1')
        print(datarow, end = "")
    
        # Pass this file object to csv.writer()
        # and get a writer object
        writer_object = writer(f_object)
    
        # Pass the list as an argument into
        # the writerow()
        writer_object.writerow(datarow)
                
 
if __name__ == "__main__":
    
    try: 
        main()
        
    except KeyboardInterrupt:
        ser.close()
        f_object.close()
        print("============")
        print("\nKilled.")
        print("============")
    
    except:
        ser.close()
        f_object.close()
