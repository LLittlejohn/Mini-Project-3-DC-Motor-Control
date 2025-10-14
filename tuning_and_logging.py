import sys
import time
from serial import Serial
import math
import matplotlib.pyplot as plt
import numpy as np
import threading

var_list = {
    "NONE":0, 
    "LINEAR":1, 
    "ANGULAR":2, 
    "LEFT_BIAS":3, 
    "KP":4, 
    "KI":5, 
    "KD":6, 
    "STOP":7
}

class TuningHelper():
    def __init__(self,connection):
        self.cxn = connection

    def main_loop(self):
        while True:
            print("Enter the number of the variable to tune:")
            for name,num in var_list.items():
                print(f"{name}: {num}")
            user_in_choice = input("Choice: ")
            print("got choice")
            try:
                if int(user_in_choice) not in {0,1,2,3,4,5,6,7}:
                    continue
            except ValueError:
                continue

            user_in_value = input("Enter the float value to pass to the variable: ")
            try:
                user_in_float = float(user_in_value)
            except ValueError:
                print("Entered value must be convertible to a float")
                continue

            # Pass value over serial
            print("about to pass")
            ascii_line = f"{user_in_choice}\n".encode("ascii")
            self.cxn.write(ascii_line)
            self.cxn.flush()

            print("passed one value")
            ascii_line = f"{user_in_float}\n".encode("ascii")
            self.cxn.write(ascii_line)
            self.cxn.flush()
            print("passed both values")

class VisualLogger:
    """
    Class to record and display data logged over the serial connection

    Attributes: 
        verbose: whether to print verbose output from serial read
        cxn: serial connection object
        var_list: enum of variable names versus integer id
        num_log_fields: how many fields are passed over serial
        measurement_history: how many measurements to display at once
        iteration: current iteration of the serial data reading, modulo measurement_history
    """
    def __init__(self,cxn,verbose,spoof_data):
        self.verbose = verbose

        if not spoof_data:
            self.cxn = cxn
        else:
            self.cxn = None

        self.var_list = {
            "NONE":0, 
            "LINEAR":1, 
            "ANGULAR":2, 
            "LEFT_BIAS":3, 
            "KP":4, 
            "KI":5, 
            "KD":6, 
            "CONTROL_MODE":7
        }
        self.num_log_fields = 8
        self.measurement_history = 500
        self.iteration = 0

        self.reading_l_arr = np.zeros((self.measurement_history * 2),dtype=int)
        self.reading_r_arr = np.zeros((self.measurement_history * 2),dtype=int)
        self.motor_l_arr = np.zeros((self.measurement_history * 2),dtype=float)
        self.motor_r_arr = np.zeros((self.measurement_history * 2),dtype=float)
        self.kp_arr = np.zeros((self.measurement_history * 2),dtype=float)
        self.ki_arr = np.zeros((self.measurement_history * 2),dtype=float)
        self.kd_arr = np.zeros((self.measurement_history * 2),dtype=float)
        self.timestamps = np.zeros((self.measurement_history * 2),dtype=int)

        self.init_plot()

    def init_plot(self):
        """ Initialize plots """
        self.fig,self.ax = plt.subplots(nrows=3,ncols=1)

        self.line_rl = self.ax[0].plot([],[])[0]
        self.line_rr = self.ax[0].plot([],[])[0]

        self.line_ml = self.ax[1].plot([],[])[0]
        self.line_mr = self.ax[1].plot([],[])[0]

        self.line_kp = self.ax[2].plot([],[])[0]
        self.line_ki = self.ax[2].plot([],[])[0]
        self.line_kd = self.ax[2].plot([],[])[0]

        self.ax[0].set_title("Left and right sensor readings over time")
        self.ax[0].set_xlabel("Time (ms)")
        self.ax[0].set_ylabel("Reading (0-1023)")
        self.ax[0].legend(loc='upper right', labels=["Left","Right"])

        self.ax[1].set_title("Left and right motor commands over time")
        self.ax[1].set_xlabel("Time (ms)")
        self.ax[1].set_ylabel("Motor speed (0-256)")
        self.ax[1].legend(loc='upper right', labels=["Left","Right"])

        self.ax[2].set_title("kP, kI, and kD over time")
        self.ax[2].set_xlabel("Time (ms)")
        self.ax[2].set_ylabel("Coefficient Value")
        self.ax[2].legend(loc='upper right', labels=["kP","kI","kD"])
        
        plt.tight_layout()
        plt.show(block=False)

    def main_loop(self):
        """ Main loop to listen to and display data """
        while True:
            #print("start loop")
            result = self.cxn.readline().decode("ascii").strip()

            if len(result) < 1:  # Ignore incomplete data
                continue

            result_split = result.split(",")

            # Ignore incomplete data
            if len(result_split) < self.num_log_fields or "" in result_split:
                continue

            # Useful information in case data processing is wrong
            if self.verbose:
                print(type(result))
                print(result)
                print(result_split)
                print(type(result_split[0]))

            # Arduino script indicates scan is complete by passing
            # negatives; break out of loop upon receiving these
            if "-1" in result_split:
                return

            reading_l = int(result_split[0])
            reading_r = int(result_split[1])
            motor_l = float(result_split[2])
            motor_r = float(result_split[3])
            kp = float(result_split[4])
            ki = float(result_split[5])
            kd = float(result_split[6])
            timestamp = int(result_split[7])

            self.display_data(reading_l,reading_r,motor_l,motor_r,kp,ki,kd,timestamp)
            self.iteration += 1
            self.iteration %= self.measurement_history

    def display_data(self,rl,rr,ml,mr,kp,ki,kd,timestamp):
        """
        Plots the most recent window of sensor readings via Matplotlib,
        and inserts each value in the correct array position
        Takes in a single value for each parameter over serial

        Args:   
            rl: left sensor reading
            rr: right sensor reading
            ml: left motor command
            mr: right motor command
            kp: proportional coefficient
            ki: integral coefficient
            kd: derivative coefficient
            timestamp: measurement time, in ms
        """
        start = self.iteration
        end = self.iteration + self.measurement_history

        self.reading_l_arr[start] = rl
        self.reading_r_arr[start] = rr
        self.motor_l_arr[start] = ml
        self.motor_r_arr[start] = mr
        self.kp_arr[start] = kp
        self.ki_arr[start] = ki
        self.kd_arr[start] = kd
        self.timestamps[start] = timestamp

        self.reading_l_arr[end] = rl
        self.reading_r_arr[end] = rr
        self.motor_l_arr[end] = ml
        self.motor_r_arr[end] = mr
        self.kp_arr[end] = kp
        self.ki_arr[end] = ki
        self.kd_arr[end] = kd
        self.timestamps[end] = timestamp

        t = self.timestamps[start+1:end]

        self.line_rl.set_data(t,self.reading_l_arr[start+1:end])
        self.line_rr.set_data(t,self.reading_r_arr[start+1:end])

        self.line_ml.set_data(t,self.motor_l_arr[start+1:end])
        self.line_mr.set_data(t,self.motor_r_arr[start+1:end])

        self.line_kp.set_data(t,self.kp_arr[start+1:end])
        self.line_ki.set_data(t,self.ki_arr[start+1:end])
        self.line_kd.set_data(t,self.kd_arr[start+1:end])

        for a in self.ax:
            a.relim()
            a.autoscale_view()

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def spoof_data(self):
        """
        Manually generate data to test plotting in case the Arduino
        connection isn't available. 
        """
        count = 0
        while True:
            try:
                self.display_data(
                    100 * math.sin(count / 13),
                    100 * math.cos(count / 13),
                    math.sin(count / 10) + math.cos(count / 13),
                    math.sin(count / 10) - math.cos(count / 13),
                    count // 31,
                    count // 21,
                    count // 11,
                    count)
                self.iteration += 1
                count += 1
                self.iteration %= self.measurement_history
            except KeyboardInterrupt:
                sys.exit(0)

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
    spoof_data = False # Change to True if testing visualization
    cxn = init_connection()
    tuner = TuningHelper(connection=cxn)
    logger = VisualLogger(cxn=cxn,verbose=True,spoof_data=spoof_data)

    tune_thread = threading.Thread(target=tuner.main_loop,daemon=True)

    try:
        tune_thread.start()
        while True:
            #logger.main_loop()
            pass
    except KeyboardInterrupt:
        tune_thread.join()
        sys.exit()
