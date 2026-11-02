from rclpy.time import Time
from utilities import Logger

# Controller type
P=0 # poportional
PD=1 # proportional and derivative
PI=2 # proportional and integral
PID=3 # proportional, integral, derivative

# For the TODO items in this file, you can utilize your implementation from LAB-2 (if it was properly implemented)
class PID_ctrl:
    
    def __init__(self, type_, kp=1.2,kv=0.8,ki=0.2, history_length=3, filename_="errors.csv"):
        
        # Data for the controller
        self.history_length=history_length
        self.history=[]
        self.type=type_

        # Controller gains
        self.kp=kp
        self.kv=kv
        self.ki=ki
        
        self.logger=Logger(filename_)

    
    def update(self, stamped_error, status):
        
        if status == False:
            self.__update(stamped_error)
            return 0,0
        else:
            return self.__update(stamped_error)

        
    def __update(self, stamped_error):
        
        latest_error=stamped_error[0]
        stamp=stamped_error[1]
        
        self.history.append(stamped_error)        
        
        if (len(self.history) > self.history_length):
            self.history.pop(0)
        
        # If insufficient data points, use only the proportional gain
        if (len(self.history) != self.history_length):
            return self.kp * latest_error
        
        # Compute the error derivative
        dt_avg=0
        error_dot=0
        
        for i in range(1, len(self.history)):
            
            t0=Time.from_msg(self.history[i-1][1])
            t1=Time.from_msg(self.history[i][1])
            
            dt=(t1.nanoseconds - t0.nanoseconds) / 1e9
            
            dt_avg+=dt            
            
            # calculate the error dot 
            # the dt sometimes happen to be zero or very small due to the 
            # connection issues to prevent the zero division error
            dt =0.1
            error_dot+=(self.history[i][0] - self.history[i-1][0])/dt
            
        error_dot/=len(self.history)
        dt_avg/=len(self.history)
        
        # Compute the error integral
        sum_=0
        for hist in self.history:
            # Gather the integration
            sum_+=hist[0] 
        
        error_int=sum_*dt_avg
        
        # Log your errors
        self.logger.log_values( [latest_error, error_dot, error_int, Time.from_msg(stamp).nanoseconds])

        # Control law corresponding to each type of controller
        if self.type == P:
            return self.kp * latest_error
        
        elif self.type == PD:
            return self.kp * latest_error + self.kv * error_dot
        
        elif self.type == PI:
            return self.kp * latest_error +  self.ki * error_int
        
        elif self.type == PID:
            
            return self.kp * latest_error + self.kv * error_dot + self.ki * error_int