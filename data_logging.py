""" Logging functionality to record and display relevant line follow values """
import sys
import math
import matplotlib.pyplot as plt
from serial import Serial
import numpy as np

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
    def __init__(self,verbose,spoof_data):
        self.verbose = verbose

        # Initiate serial connection with Arduino
        if not spoof_data:
            self.cxn = self.init_connection()
        else:
            self.cxn = None

        # IDs of parameters to tune (matches up with enum for tuning on the Arduino)
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

        self.num_log_fields = 8 # Number of fields passed over serial when logging
        self.measurement_history = 500 # Number of values to plot at once
        self.iteration = 0

        # Initialize arrays to store readings
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
        """
        Initialize plots to visualize data
        Plots are updated by changing the data each line represents
        """
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
            try:
                # Read input on serial
                result = self.cxn.readline().decode("ascii").strip()

                if len(result) < 1:  # Ignore empty data
                    continue
                result_split = result.split(",")

                # Ignore data that isn't the number of fields we expect
                if len(result_split) < self.num_log_fields or "" in result_split:
                    continue

                # Useful information in case data processing is wrong
                if self.verbose:
                    print(type(result))
                    print(result)
                    print(result_split)
                    print(type(result_split[0]))

                # Cast passed fields to correct data types
                reading_l = int(result_split[0])
                reading_r = int(result_split[1])
                motor_l = float(result_split[2])
                motor_r = float(result_split[3])
                kp = float(result_split[4])
                ki = float(result_split[5])
                kd = float(result_split[6])
                timestamp = int(result_split[7])

                # Display correct data given the iteration
                self.display_data(reading_l,reading_r,motor_l,motor_r,kp,ki,kd,timestamp)
                self.iteration += 1
                self.iteration %= self.measurement_history

            except KeyboardInterrupt:
                print("Keyboard Interrupt received, exiting")
                sys.exit(0)

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

        # Add data to array
        self.reading_l_arr[start] = rl
        self.reading_r_arr[start] = rr
        self.motor_l_arr[start] = ml
        self.motor_r_arr[start] = mr
        self.kp_arr[start] = kp
        self.ki_arr[start] = ki
        self.kd_arr[start] = kd
        self.timestamps[start] = timestamp

        # Add data to shifted indices of array
        self.reading_l_arr[end] = rl
        self.reading_r_arr[end] = rr
        self.motor_l_arr[end] = ml
        self.motor_r_arr[end] = mr
        self.kp_arr[end] = kp
        self.ki_arr[end] = ki
        self.kd_arr[end] = kd
        self.timestamps[end] = timestamp

        t = self.timestamps[start+1:end]

        # Change data in plotted lines to reflect new data
        self.line_rl.set_data(t,self.reading_l_arr[start+1:end])
        self.line_rr.set_data(t,self.reading_r_arr[start+1:end])

        self.line_ml.set_data(t,self.motor_l_arr[start+1:end])
        self.line_mr.set_data(t,self.motor_r_arr[start+1:end])

        self.line_kp.set_data(t,self.kp_arr[start+1:end])
        self.line_ki.set_data(t,self.ki_arr[start+1:end])
        self.line_kd.set_data(t,self.kd_arr[start+1:end])

        # Rescale axes to show data properly
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

    def init_connection(self):
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
    spoof_data = False # Change to True if testing visualization
    if spoof_data:
        logger = VisualLogger(verbose=True,spoof_data=True)
        logger.spoof_data()
    else:
        logger = VisualLogger(verbose=True,spoof_data=False)
        logger.main_loop()
