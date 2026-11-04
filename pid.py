from rclpy.time import Time
from utilities import Logger

# Controller type
P=0 # poportional
PD=1 # proportional and derivative
PI=2 # proportional and integral
PID=3 # proportional, integral, derivative

class PID_ctrl:
    # def __init__(self, type_, kp=1.2,kv=0.8,ki=0.2, history_length=3, filename_="errors.csv"): PID1
    # def __init__(self, type_, kp=1.2,kv=1.6,ki=0.2, history_length=3, filename_="errors.csv"): PID2
    # def __init__(self, type_, kp=2.2,kv=1.6,ki=0.2, history_length=3, filename_="errors.csv"): PID3
    # def __init__(self, type_, kp=3.2,kv=2.6,ki=0.2, history_length=3, filename_="errors.csv"): PID4 
    def __init__(self, type_, kp=1.2,kv=0.8,ki=0.2, history_length=3, filename_="errors.csv"): 
        
        # Data for the controller
        self.history_length=history_length
        self.history=[]
        self.integral = 0.0
        self.previous = None
        self.type=type_

        # Controller gains
        self.kp=kp    # proportional gain
        self.kv=kv    # derivative gain
        self.ki=ki    # integral gain
        
        self.logger=Logger(filename_)
        # Remeber that you are writing to the file named filename_ or errors.csv the following:
            # error, error_dot, error_int and time stamp

    
    def update(self, stamped_error, status):
        
        if status == False:
            self.__update(stamped_error)
            return 0.0
        else:
            return self.__update(stamped_error)

        
    def __update(self, stamped_error):
        
        latest_error=stamped_error[0]
        stamp=stamped_error[1]
        
        stamp_seconds = Time.from_msg(stamp).nanoseconds / 1e9
        error_dot = 0.0
        if self.previous is not None:
            old_error, old_time = self.previous
            dt = stamp_seconds - old_time
            if dt <= 0:
                # Ignore duplicate/out-of-order samples without changing PID state.
                return getattr(self, "last_output", self.kp * latest_error)
            if dt <= 0.5:
                error_dot = (latest_error - old_error) / dt
                # Trapezoidal integral with a bound on its output contribution.
                self.integral += 0.5 * (latest_error + old_error) * dt
                if self.ki:
                    bound = 0.1 / abs(self.ki)
                    self.integral = max(-bound, min(bound, self.integral))
            else:
                self.integral = 0.0
        self.previous = (latest_error, stamp_seconds)
        error_int = self.integral

        # TODONE Part 4: Log your errors

        #collect the time stamp in seconds, latest error, error dot, and error integral and log them to the file
        stamp_seconds = Time.from_msg(stamp).nanoseconds / 1e9
        self.logger.log_values( [
            latest_error, error_dot, error_int, stamp_seconds
        ] )
        
        output = self.kp * latest_error
        if self.type in (PD, PID):
            output += self.kv * error_dot
        if self.type in (PI, PID):
            output += self.ki * error_int
        self.last_output = output
        return output
