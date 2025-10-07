import sys
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
PRINT_DEBUG = True

def main():
    cxn = init_connection()

    reading_l = []
    reading_r = []
    motor_l = []
    motor_r = []
    is_waiting = True  # Wait for user input before starting scan

    while True:
        print("start loop")
        result = cxn.readline().decode("ascii").strip()

        if len(result) < 1:  # Ignore incomplete data
            continue

        result_split = result.split(",")

        # Ignore incomplete data
        if len(result_split) < 3 or "" in result_split:
            continue

        # Useful information in case data processing is wrong
        if PRINT_DEBUG:
            print(type(result))
            print(result)
            print(result_split)
            print(type(result_split[0]))

        # Arduino script indicates scan is complete by passing
        # negatives; break out of loop upon receiving these
        if "-1" in result_split:
            break

        # Append readings to lists
        # Preallocating would be slightly faster but isn't
        # strictly necessary
        reading_l.append(int(result_split[0]))
        reading_r.append(int(result_split[1]))
        motor_l.append(int(result_split[2]))
        motor_r.append(int(result_split[3]))

def init_connection():
    """
    Initialize serial connection with a port specified by the first argument
    passed when calling the python file.
    """
    port = "/dev/cu.usbmodem1051DB37DE2C2"
    if len(sys.argv) > 1:
        port = sys.argv[1]
    cxn = Serial(port, baudrate=9600)
    cxn.write([int(1)])
    return cxn

if __name__ == "__main__":
    main()
