# Gorur Gari - Autonomous Vehicle Documentation

## Overview
Gorur Gari is an autonomous vehicle project that combines computer vision with motor control and sensor fusion. The system uses a Raspberry Pi for vision processing and an Arduino Nano for motor control and sensor data collection.

## Project Structure

### Folder Structure
```
/
  -firmware
    -arduino_firmware.ino    # Arduino control code
  -logic
    -vision_processing.py    # Computer vision code
```

## Hardware Components

1. **Motors**:
   - JGA-25 370 motor (x2) [AliExpress Link](https://www.aliexpress.com/item/32844684605.html)
   - Wheel radius: 0.0325m (65mm diameter)

2. **Control Board**:
   - Arduino Nano (for motor control and sensor interfacing)

3. **Sensors**:
   - Ultrasonic sensors (x4) for obstacle detection
   - Quadrature encoders for wheel rotation tracking

4. **Vision System**:
   - Raspberry Pi 4
   - Camera module (connected via /dev/video2)

## Firmware Documentation (arduino_firmware.ino)

### Key Features
- Motor control with PWM
- Quadrature encoder reading for odometry
- Ultrasonic distance measurement
- Basic odometry calculations

### Pin Configuration
| Component        | Arduino Pin |
|------------------|-------------|
| Encoder A        | 2           |
| Encoder B        | 3           |
| Ultrasonic 1 Trig| 4           |
| Ultrasonic 1 Echo| 5           |
| Ultrasonic 2 Trig| 6           |
| Ultrasonic 2 Echo| 7           |
| Ultrasonic 3 Trig| 8           |
| Ultrasonic 3 Echo| 9           |
| Ultrasonic 4 Trig| 10          |
| Ultrasonic 4 Echo| 11          |
| Left Motor       | 10          |
| Right Motor      | 11          |

### Odometry Parameters
- Wheel radius: 0.0325m
- Counts per revolution: 44 (11 PPR × 4 for quadrature decoding)
- Meters per count: 0.00464m

### Key Functions
1. **`updateEncoder()`** - Interrupt service routine for quadrature decoding
2. **`getUSReading()`** - Gets distance from ultrasonic sensor
3. **`controlMotor()`** - Controls motor speed and direction
4. **`updatePosition()`** - Updates robot position (simplified model)

## Vision System Documentation (vision_processing.py)

### Key Features
- Color-based object detection (red and green)
- Distance estimation using object height
- Camera control and frame processing

### Color Detection Parameters
- **Red**:
  - Lower range 1: [0, 100, 100]
  - Upper range 1: [10, 255, 255]
  - Lower range 2: [160, 100, 100]
  - Upper range 2: [179, 255, 255]
  
- **Green**:
  - Lower range: [36, 100, 50]
  - Upper range: [85, 255, 255]

### Distance Estimation
Uses quadratic formula based on object height:
`distance = h² × 0.001543 - 1.313 × h + 324.53` (where h is object height in pixels)

### Key Functions
1. **`estimate_distance()`** - Estimates distance to detected object
2. **`get_color_mask()`** - Creates HSV masks for color detection
3. **`find_boxes()`** - Finds rectangular objects in the frame

## Communication Protocol

The system uses serial communication between Raspberry Pi and Arduino. The current implementation prints sensor data in CSV format:

```
<encoder_count>,<US1_distance>,<US2_distance>,<US3_distance>
```

## Setup Instructions

1. **Hardware Assembly**:
   - Connect motors to designated pins
   - Mount ultrasonic sensors facing front, back, left and right
   - Connect encoders to interrupt-capable pins (2 and 3)
   - Connect Raspberry Pi camera

2. **Software Setup**:
   - Upload Arduino firmware
   - Install Python dependencies on Raspberry Pi:
     ```
     pip install opencv-python numpy
     ```
   - Run vision processing script:
     ```
     python3 logic/vision_processing.py
     ```

## Future Improvements

1. Implement proper serial communication protocol between Pi and Arduino
2. Add PID control for motor speed regulation
3. Implement more accurate odometry with both wheels
4. Add SLAM capabilities
5. Improve distance estimation with camera calibration

## Troubleshooting

1. **Camera not detected**:
   - Check if camera is properly connected
   - Verify correct video device path in code

2. **Motor not responding**:
   - Check PWM pin connections
   - Verify motor power supply

3. **Ultrasonic sensors giving erratic readings**:
   - Ensure sensors have clear line of sight
   - Check for electrical interference

4. **Encoder counts not updating**:
   - Verify interrupt pins are correctly connected
   - Check for proper pull-up resistors