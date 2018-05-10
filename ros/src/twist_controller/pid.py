
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        # Class member variables
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = 0.
        self.prev_int_val = 0.
        self.last_error = 0.

    def reset(self):
        """Reset PID controller variables

        """
        self.int_val = 0.0
        self.prev_int_val = 0.0

    def step(self, error, sample_time):
        """Adjust controller values based on updated error & elapsed time

        """
        self.prev_int_val = self.int_val
        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        val = self.kp * error + self.ki * self.int_val + self.kd * derivative
        val = max(self.min, min(val, self.max))

        self.int_val = integral
        self.last_error = error

        return val
