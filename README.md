# Motion Detection Using Python and OpenCV

## Introduction
This personal project is a motion detection system developed using Python and OpenCV. It's crafted to detect and track motion in video streams, employing the Kalman Filter for improved tracking accuracy. This system is ideal for personal applications such as home surveillance, hobbyist projects, or learning exercises in image processing and computer vision.

## Features
- Motion detection in video streams in real-time.
- Kalman Filter integration for advanced object tracking.
- Adjustable tracking and detection parameters.
- Simple graphical interface for video playback control.
- Utilization of OpenCV for efficient video frame processing.

## Requirements
- Python 3.9 or higher
- OpenCV (cv2)
- Numpy
- Matplotlib
- Skvideo
- Scikit-image
- Tkinter
- PIL


## How It Works
1. **Video Playback:** The system uses Tkinter for the user interface and OpenCV for processing video frames.
2. **Motion Detection:** Detects motion by analyzing changes between consecutive frames.
3. **Kalman Filter:** Employs a Kalman Filter to predict the position of moving objects, enhancing tracking accuracy.
4. **Object Tracking:** Capable of tracking multiple objects simultaneously.

## Configuration

- `skip_frames`: Number of frames to skip for detection. A lower number increases detection frequency.
- `N`: The maximum number of objects that can be tracked at once.
- Kalman Filter parameters like `x`, `E`, `D`, `B`, `H`, `R`, `u` for fine-tuning the tracking algorithm.


## Copyright
Copyright © 2024. All rights reserved.

This software is provided for educational purposes only. It is prohibited to use this for any college assignment or personal use. Unauthorized distribution, modification or commercial usage is strictly forbidden. Please respect the rights of the author.
