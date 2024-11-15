import csv
from pid_controller import PIDController  # Assuming PIDController is in pid_controller.py
import random

def export_pid_parameters(Kp, Ki, Kd, setpoint):
    """Export the PID parameters to a CSV file."""
    with open('pid_parameters.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Parameter', 'Value'])
        writer.writerow(['Kp', Kp])
        writer.writerow(['Ki', Ki])
        writer.writerow(['Kd', Kd])
        writer.writerow(['Setpoint', setpoint])
    print("PID parameters saved to 'pid_parameters.csv'.")

def run_pid_simulation():
    # PID parameters
    Kp = 10.0
    Ki = 0.3
    Kd = 0.05
    setpoint = 20

    # Export PID parameters to a separate CSV file
    export_pid_parameters(Kp, Ki, Kd, setpoint)

    # Set up the PID controller with chosen gains and setpoint
    pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd, setpoint=setpoint)

    # Initialize simulation variables
    measured_value = 0  # Initial position (could represent any process variable)
    time_step = 0.1     # Time interval between updates (in seconds)
    total_time = 10     # Total simulation time (in seconds)
    time = 0            # Initialize time

    # Prepare the CSV file for output
    with open('pid_output.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'Measured Value', 'Control Output'])

        # Run the simulation loop
        while time <= total_time:
            # Compute control output from the PID controller
            control_output = pid.update(measured_value)
            
            # Simulate the effect of the control output on the measured value
            # Here we add some randomness to simulate a process responding to the control signal.
            measured_value += control_output * time_step + random.uniform(-0.1, 0.1)

            # Write the current time, measured value, and control output to the CSV file
            writer.writerow([time, measured_value, control_output])

            # Increment time
            time += time_step

    print("Simulation complete. Results saved to 'pid_output.csv'.")

if __name__ == "__main__":
    run_pid_simulation()
