# HandSpeak: Gesture-to-Audio Conversion System

## Overview
HandSpeak is an assistive technology project that converts hand gestures into audio feedback using a combination of flex sensors and motion sensing. This system enables non-verbal communication through customized hand gestures that trigger corresponding audio messages.

## Hardware Components
- ESP32 Microcontroller
- 5 flex sensors (one for each finger)
- MPU6050 accelerometer/gyroscope module
- DFPlayer Mini MP3 player module
- Speaker
- Power supply
- Connecting wires and breadboard

## Features
- Real-time hand gesture recognition
- Customizable gesture-to-audio mapping
- Low latency response time
- Support for multiple predefined audio messages

## How It Works
1. Flex sensors mounted on a glove detect finger bending angles
2. The MPU6050 tracks hand orientation and movement
3. An ESP32 processes the combined sensor data to identify specific gestures
4. When a recognized gesture is detected, the DFPlayer Mini plays the corresponding audio file
5. The system provides immediate audio feedback through the connected speaker

## Installation and Setup
1. Connect the hardware components according to the circuit diagram
2. Upload the provided ESP32 code to your microcontroller using ESP-IDF
3. Place audio files (in MP3 format) on the microSD card for the DFPlayer Mini
4. Mount the sensors on a glove or wearable structure
5. Calibrate the system for your specific hand movements

## Future Development Plans

### Machine Learning Integration
- Implement neural networks for more accurate gesture recognition
- Enable learning of new gestures through training mode
- Add support for dynamic gestures and sequences
- Incorporate adaptive algorithms to adjust to user's specific movement patterns

### Hardware Enhancements
- Design custom PCB to replace breadboard connections
- Develop wireless communication capabilities
- Miniaturize components for improved wearability
- Add haptic feedback for user confirmation
- Implement rechargeable battery solution


## Project Demonstration
Watch the demonstration video:
[YouTube Link](<https://youtube.com/shorts/xSCXOc_uqc0?feature=share>) 

## Credits
This project was developed as a solution to enhance communication capabilities for individuals who use sign language or have speech impairments.