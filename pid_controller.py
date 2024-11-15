# pid_controller.py

class PIDController: # This is a test
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        """Initialize PID controller with given gains and setpoint."""
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, measured_value):
        """Calculate control output based on the measured value."""
        error = self.setpoint - measured_value
        self.integral += error
        derivative = error - self.previous_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output
