from sweeppy import Sweep

# Replace '/dev/ttyUSB0' with the actual port your LIDAR is connected to
SWEEP_PORT = '/dev/ttyUSB0'

def get_lidar_data():
    """
    Connects to the Sweep LIDAR, gets a single scan, and prints the data.
    """
    with Sweep(SWEEP_PORT) as sweep:
        # Set the motor speed (e.g., to 5 Hz)
        sweep.set_motor_speed(5) 
        
        # Start the scanner
        sweep.start_scanning()
        
        print(f"Motor Speed: {sweep.get_motor_speed()} Hz")
        print(f"Sample Rate: {sweep.get_sample_rate()} Hz")

        # Get the first full 360-degree scan
        for scan in sweep.get_scans():
            print(f"Received scan with {len(scan.samples)} samples.")
            
            # Iterate through all samples in the scan
            for sample in scan.samples:
                angle_degrees = sample.angle / 1000 # Angle is in milli-degrees
                distance_cm = sample.distance
                signal_strength = sample.signal_strength
                
                # You can use this data for processing, e.g., publishing to ROS 2
                # print(f"Angle: {angle_degrees:.2f}° | Distance: {distance_cm} cm | Signal: {signal_strength}")
            
            # Break after the first scan for a simple example
            break

        # Stop the scanner
        sweep.stop_scanning()

if __name__ == '__main__':
    get_lidar_data()
