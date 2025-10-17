import sys
import time
from serial import Serial, SerialException

var_list = {
    "NONE":0, 
    "LINEAR":1, 
    "ANGULAR":2, 
    "LEFT_BIAS":3, 
    "KP":4, 
    "KI":5, 
    "KD":6, 
    "CONTROL_MODE":7
}

def main():
    cxn = init_connection()
    while True:
        print("Enter the number of the variable to tune:")
        for name,num in var_list.items():
            print(f"{name}: {num}")
        try:
            user_in_choice = int(input("Choice: "))
        except ValueError:
            continue

        if user_in_choice not in {0,1,2,3,4,5,6,7}:
            continue

        user_in_value = input("Enter the float value to pass to the variable: ")
        try:
            user_in_float = float(user_in_value)
        except ValueError:
            print("Entered value must be convertible to a float")
            continue

        # Pass value over serial
        line = f"{user_in_choice}\r\n".encode("ascii")
        cxn.write(line)  # Begins the servo movement
        cxn.flush()
        time.sleep(0.1)
        print(f"result: {cxn.readline().decode('utf-8').strip()}")
        line = f"{user_in_float}\r\n".encode("ascii")
        cxn.write(line)  # Begins the servo movement
        cxn.flush()
        print(f"result: {cxn.readline().decode('utf-8').strip()}")
        for i in range(10):
            print(f"result: {cxn.readline().decode('utf-8').strip()}")
            time.sleep(0.01)

def init_connection():
    """
    Initialize serial connection with a port specified by the first argument
    passed when calling the python file.
    """
    port = "/dev/cu.usbmodem1051DB37DE2C2"
    if len(sys.argv) > 1:
        port = sys.argv[1]
    cxn = Serial(port, baudrate=9600)
    time.sleep(1.0)
    return cxn

if __name__ == "__main__":
    main()
