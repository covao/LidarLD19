import serial
import numpy as np
import matplotlib.pyplot as plt
import time

# SerialRead Class
class SerialRead:
    '''
    Serial Communication Class for reading data from a UART port.
    '''
    
    def __init__(self, port_name='/dev/ttyUSB0', baud_rate=230400):
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.ser = None
        self.open_serial()

    def open_serial(self):
        '''
        Open the serial port.
        '''
        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=1)
        except Exception as e:
            print(f"Error opening serial port: {e}")
    
    def run_threaded(self):
        '''
        Read all available data from the serial port and return it as a byte array.
        '''
        if self.ser is not None and self.ser.is_open:
            #data = self.ser.read(1024)
            data = self.ser.read_all()
            return data
        return bytearray()

    def shutdown(self):
        '''
        Close the serial port when the program is terminated.
        '''
        if self.ser is not None and self.ser.is_open:
            self.ser.close()


# LD19Lidar Parse Class
class LD19LidarParse:
    '''
    Lidar Packet Parsing Class
    Parses the LD19 Lidar packets and extracts distance, intensity, and angle data.
    '''

    def __init__(self):
        # Buffer for storing incomplete data
        self.buffer = bytearray()

    def run_threaded(self, byte_data, reset=0):
        if reset > 0:
            self.buffer = bytearray()  # Reset buffer if necessary
        
        # Add incoming byte data to buffer
        self.buffer += byte_data

        distance_data = []
        intensity_data = []
        angle_data = []

        while len(self.buffer) >= 47:  # 47 bytes minimum per packet
            # Search for header (0x54)
            header_index = self.buffer.find(b'\x54')
            if header_index == -1:
                # If header not found, clear the buffer
                self.buffer = bytearray()
                break

            # Check if enough data for a full packet
            if len(self.buffer) < header_index + 47:
                # If not enough data, wait for more
                break

            # Extract packet data
            packet = self.buffer[header_index:header_index + 47]
            self.buffer = self.buffer[header_index + 47:]  # Remove processed data

            # Parse the packet fields
            speed = int.from_bytes(packet[2:4], byteorder='little')  # Speed in degrees/second
            start_angle = int.from_bytes(packet[4:6], byteorder='little') / 100.0  # Start angle in degrees
            end_angle = int.from_bytes(packet[42:44], byteorder='little') / 100.0  # End angle in degrees

            # Calculate angle increment per point
            point_count = 12  # 12 data points in the packet
            
            # Calculate angle increment per point with wrap-around handling
            angle_diff = (end_angle - start_angle) % 360.0
            angle_increment = angle_diff / (point_count - 1)
    
            # Extract distance and intensity data
            for i in range(point_count):
                index = 6 + i * 3
                distance = int.from_bytes(packet[index:index + 2], byteorder='little')  # Distance in mm
                intensity = packet[index + 2]  # Intensity
                angle = start_angle + i * angle_increment  # Calculate angle for each point

                distance_data.append(distance)
                intensity_data.append(intensity)
                angle_data.append(angle)

        return distance_data, intensity_data, angle_data


class NormalizeScan:
    '''
    Normalize Lidar Scan Data
    Converts angle, distance, and intensity arrays into distance and intensity data per step angle (0-359 degrees).
    '''
    
    def __init__(self, step_angle=1, max_distance=20000, min_intensity=5, min_angle=0, max_angle=359):
        self.step_angle = step_angle  # Step size in degrees
        self.max_distance = max_distance  # Maximum distance in mm
        self.min_intensity = min_intensity  # Minimum intensity threshold
        self.min_angle = min_angle  # Minimum output angle in degrees
        self.max_angle = max_angle  # Maximum output angle in degrees
        self.angle_steps = int(360 / self.step_angle)  # Total number of steps (0-359 degrees)
        self.previous_distances = [self.max_distance] * self.angle_steps  # Initialize with max_distance
        self.previous_intensities = [0] * self.angle_steps  # Initialize intensities with 0

    def run_threaded(self, angles, distances, intensities, reset=1):
        '''
        Normalize the scan data by processing angles, distances, and intensities.
        It converts the data into step-wise distance and intensity arrays.
        '''
        # Reset the distance and intensity arrays if reset flag is on
        if reset:
            self.previous_distances = [self.max_distance] * self.angle_steps
            self.previous_intensities = [0] * self.angle_steps

        # Create new arrays initialized with the previous values
        normalized_distances = self.previous_distances[:]
        normalized_intensities = self.previous_intensities[:]

        # Normalize angles to be within 0-359 degrees
        normalized_angles_input = np.mod(angles, 360)

        # Convert angles to step indices and assign distances and intensities
        for angle, distance, intensity in zip(normalized_angles_input, distances, intensities):
            step_index = int(angle // self.step_angle)
            if 0 <= step_index < self.angle_steps:
                # If the intensity is less than the minimum, set distance to max_distance
                if intensity < self.min_intensity:
                    normalized_distances[step_index] = self.max_distance
                    normalized_intensities[step_index] = 0
                else:
                    # Set distance to max_distance if distance is 0
                    if distance == 0:
                        normalized_distances[step_index] = self.max_distance
                    else:
                        normalized_distances[step_index] = distance
                    normalized_intensities[step_index] = intensity

        # Prepare output angles and filter them within the specified output range
        normalized_angles = list(range(self.min_angle, self.max_angle + 1, self.step_angle))

        # Filter the distances and intensities within the specified angle range
        start_index = self.min_angle // self.step_angle
        end_index = (self.max_angle // self.step_angle) + 1
        filtered_distances = normalized_distances[start_index:end_index]
        filtered_intensities = normalized_intensities[start_index:end_index]

        # Update the previous distances and intensities for future runs
        self.previous_distances = normalized_distances[:]
        self.previous_intensities = normalized_intensities[:]

        return normalized_angles, filtered_distances, filtered_intensities


class LD19Lidar:
    '''
    Class to output distance data from the LD19 Lidar in normalized steps.
    '''
    def __init__(self, step_angle=1, min_angle=0, max_angle=359, port_name='/dev/ttyUSB0'):
        self.serial_reader = SerialRead(port_name=port_name)  # Class to read from serial port
        self.lidar_parser = LD19LidarParse()   # Class to parse LD19 Lidar data
        self.normalizer = NormalizeScan(step_angle=step_angle, min_angle=min_angle, max_angle=max_angle)  # Class to normalize Lidar data

    def run_threaded(self, reset=0):

        # Step 1: Read from serial
        serial_data = self.serial_reader.run_threaded()

        # Step 2: Parse the serial data into distances, intensities, and angles
        distances, intensities, angles = self.lidar_parser.run_threaded(serial_data, reset)
        if distances and angles and intensities:
            # Step 3: Normalize the data (distances and intensities based on angles)
            normalized_angles, normalized_distances, normalized_intensities = self.normalizer.run_threaded(angles, distances, intensities, reset)
            
            return np.array(normalized_angles), np.array(normalized_distances)
        return np.array([]), np.array([])

    def update(self):
        pass

    def shutdown(self):
        self.serial_reader.shutdown()


import matplotlib.pyplot as plt
import numpy as np

import matplotlib.pyplot as plt
import numpy as np

class LidarPlot:
    '''
    Class for real-time plotting of Lidar data.
    '''
    
    def __init__(self, max_plot_distance=10000, point_size=1, start_angle=270):
        # Setup the polar plot for real-time visualization
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.scatter = None
        self.max_plot_distance = max_plot_distance  # Limit the plot radius to 10000 mm
        self.point_size = point_size  # Set the size of the plot points
        self.start_angle = start_angle  # Starting angle for the plot in degrees (default 270)

        # Set plot radius limit and the direction for 0 degrees
        self.ax.set_ylim(0, self.max_plot_distance)  # Set plot radius limit
        self.ax.set_title('Real-time Lidar Data')

        # Adjust the starting angle for 0 degrees and ensure clockwise plotting
        # In polar coordinates, 0 degrees is typically at the right (East), but we set it to start_angle
        self.ax.set_theta_offset(np.radians(self.start_angle))
        self.ax.set_theta_direction(-1)  # Set to clockwise direction

    def run_threaded(self, angles, distances):
        '''
        Update the plot in real-time with new angles and distances.
        Only distances within max_plot_distance will be plotted.
        '''
        # Convert angles from degrees to radians for polar plotting
        valid_angles = [np.radians(angle) for angle, distance in zip(angles, distances) if distance <= self.max_plot_distance]
        valid_distances = [distance for distance in distances if distance <= self.max_plot_distance]

        if not valid_angles or not valid_distances:
            return  # No valid data to plot

        # First-time setup of scatter plot
        if self.scatter is None:
            self.scatter = self.ax.scatter(valid_angles, valid_distances, s=self.point_size, c='b', alpha=0.75)
        else:
            # Update the plot with new data
            self.scatter.set_offsets(np.c_[valid_angles, valid_distances])

        # Refresh the plot
        plt.pause(0.01)

    def shutdown(self):
        '''Close the plot window when done.'''
        plt.close(self.fig)


# Main function modified to use LD19LidarParse and LidarPlot
def main():
    lidar_out = LD19Lidar(port_name='/dev/ttyUSB0',min_angle=0,max_angle=359) # e.g. RaspberryPi OS:port_name ='/dev/ttyUSB0, Windows OS:port_name='COM10' 
    plotter = LidarPlot()

    try:
        while True:
            normalized_angles, normalized_distances = lidar_out.run_threaded()

            # Plot the normalized data
            plotter.run_threaded(normalized_angles, normalized_distances)
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        lidar_out.shutdown()
        plotter.shutdown()

if __name__ == "__main__":
    main()

