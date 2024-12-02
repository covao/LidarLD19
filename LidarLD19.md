[Japanese(Google Translate)](https://github-com.translate.goog/covao/LidarLD19/blob/main/LidarLD19.md?_x_tr_sl=en&_x_tr_tl=ja&_x_tr_hl=ja&_x_tr_pto=wapp) 

# LD19 Lidar Parts Class Specifications
- Overview: LD19 Lidar module
- https://wiki.youyeetoo.com/en/Lidar/D300

## Serial Data Reception
- Class Name: `SerialRead`
- Input: None
- Output: Byte data array
- Initial Parameters: Serial port name
- Processing:
  - Uses Python's `pySerial` to receive data via UART
  - Reads all communication data from the buffer

## LD19 Serial Communication Data Format
- Communication Protocol:
  - Uses standard UART (Universal Asynchronous Communication)
  - One-way communication, data transmission only.
  - Communication settings:
    - Port name: /dev/ttyUSB0
    - Baud rate: 230400bps
    - Data length: 8 bits
    - Stop bit: 1 bit
    - Parity: None
    - Flow control: None

- Data Format:
  - Data packet structure:
    - Header: 1 byte (fixed value 0x54)
    - VerLen: 1 byte (upper 3 bits for packet type, lower 5 bits for the number of measurement points, fixed value 0x2C)
    - Speed: 2 bytes (LiDAR rotation speed, unit: degrees/second)
    - Start Angle: 2 bytes (start angle, unit: 0.01 degrees)
    - Data: Each measurement consists of 3 bytes (2 bytes for distance, 1 byte for reflection intensity)
    - End Angle: 2 bytes (end angle, unit: 0.01 degrees)
    - Timestamp: 2 bytes (unit: ms, maximum 30000)
    - CRC: 1 byte
- Example of a data packet:
  - Example: `54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9 CD 00 E4 CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50`
    - Header: 0x54
    - Speed: 2152 degrees/second (`68 08`)
    - Start Angle: 324.27 degrees (`7E AB`)
    - End Angle: 334.7 degrees (`82 BE`)
    - Measurement data:
      - Point 1: Distance 224mm, intensity 228 (`00 E0 E4`)
      - Point 2: Distance 220mm, intensity 226 (`00 DC E2`)
      - and so on

## Lidar Packet Parsing
- Class Name: `LD19LidarParse`
- Overview: Extracts data from LD19 Lidar serial communication packets
- Input:
  - Byte data array
  - Reset
- Output: Distance data array, intensity data array, angle data array
- Parameters: None
- Processing:
  - If there is previous buffer data, prepend it to the new data
  - Search for the header `0x54` from the beginning of the data, then parse the packet
  - If the packet size is insufficient from the header, save the data in the buffer
  - Parse the received data according to the packet format and extract the distance and intensity fields
  - Use the start and end angles to estimate the angle for each field
  - Store the angle and distance data
  - CRC check is not performed

## 2D Lidar Data Normalization
- Class Name: `NormalizeScan`
- Overview: Normalize 2D Lidar data
- Input:
  - Angle array [deg]
  - Distance array [mm]
  - Intensity data array
  - Reset
- Output:
  - Normalized angle array [deg]
  - Distance array per step angle [mm]
  - Intensity data array per step angle
- Parameters:
  - Step angle [deg], default: 1
  - Maximum distance [mm], default: 20000
  - Minimum reflection intensity, default: 10
  - Output lower angle [deg]
  - Output upper angle [deg]
- Processing:
  - Convert the angle, distance, and intensity arrays to distance and intensity arrays per step angle
  - The distance array per step angle ranges from 0 to 359 [deg]
  - For elements with intensity below the minimum reflection intensity, set the distance to the maximum distance
  - If the distance is 0, set the distance to the maximum distance
  - The newly obtained distance array overwrites the previous one
  - If the reset flag is on, the previous distance array is reset
  - Retrieve data within the specified output angle range and return

## LD19 Lidar Output
- Class Name: `LD19Lidar`
- Overview: Obtain LD19 Lidar data via serial communication and output scan data
- Input: None
- Output:
  - Normalized angle array [deg] (numpy array)
  - Distance array per step angle [mm] (numpy array)
- Parameters:
  - Serial port name, e.g., '/dev/ttyUSB0', 'COM3'
  - Step angle [deg]
  - Output lower angle [deg]
  - Output upper angle [deg]
- Processing:
  - Executes `SerialRead`, `LD19LidarParse`, and `NormalizeScan` to output distance information

## Real-time Plotting of LD19 Serial Data
- Plot Lidar data in real-time
- Overview: Output scan data from the LD19 Lidar
- Class Name: `LidarPlot`
- Input:
  - Angle array [deg]
  - Distance array [mm]
- Output: None
- Parameters:
  - Maximum plot distance [mm], default: 10000
  - Point size for plotting, default: 1
  - Starting angle for circular plot [deg], default: 270
- Processing:
  - Update the circular plot in real-time
  - Plot detected angles and distances
  - Point size for the plot is set to `point_size=1`
  - Plot in a clockwise direction

## Sample Function
- Overview: Sample code for the Lidar module
- Runs the `LD19Lidar` and `LidarPlot` in the `main` function
- Serial port name: '/dev/ttyUSB0' (default for Raspberry Pi), or for Windows: e.g., 'COM3'

## Generated Code Format
- All code should be in English
- Serial communication and data processing functions should be separated, with other functions structured to be testable
- Use the Python Parts Class Template below

## DonkeyCar Python Parts Class Template
~~~ python
# Import statements should be placed if needed
# Import xxx
class MyCounter():
    '''
    Counter example class with initial parameters
    '''

    def __init__(self, Step=0):
        ''' 
        Initialize class with initial parameters.
        '''
        self.counter = 0  # Set counter to initial value
        self.step = Step

    def run_threaded(self, In_Reset=0):
        ''' 
        Function called periodically with input values then return output values.
        '''
        self.counter += self.step  # Update the counter

        if In_Reset > 0:
            self.counter = 0  # Set counter to initial value

        # Return the output values
        return self.counter

    def update(self):
        ''' 
        Do not omit this.
        '''
        pass

    def shutdown(self):
        ''' 
        Clean up and shut down any resources or processes.
        '''
        pass
~~~


