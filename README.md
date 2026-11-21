# KX3 Panadapter for the M5Stack Tab5

## Background

This is a panadapter for the Elecraft KX3 transceiver that utilizes an inexpensive M5Stack Tab5 IOT development platform. This project was inspired by the QMX panadapter created by Steffen Lav [https://github.com/SteffenLav/qmx-panadapter ](url). Steffen's project is extensive and much more than just a panadapter. My goal in this project for the KX3 is a simple, easy to use panadapter for viewing signals on the band. Unfortunately, the KX3 does not have a USB port nor its digital audio stream (which makes Steffen's QMX panadapter so elegant), as such, putting together this system for the KX3 requires an external USB audio sound interface and a USB hub, in addition to Tab5. 

## Overview

The KX3 Panadapter is an external real-time spectrum display designed to work with the Elecraft KX3 transceiver and the M5Stack Tab5 development platform. It uses the KX3's quadrature (I/Q) audio output to display a spectrum of the received RF signals surrounding the current receive frequency.

The application transforms the Tab5 into a portable panadapter, displaying signal activity across the receiver's passband and providing a scrolling waterfall display to visualize signal activity over time.

The project is implemented using the ESP-IDF development framework and runs on the ESP32-P4 processor in the M5Stack Tab5.

## System Architecture

The system consists of four primary components:

- **Elecraft KX3 transceiver** — Provides the receiver's I/Q audio output.
- **USB audio interface** — Converts the analog I/Q signals into digital audio samples.
- **USB hub** — Provides USB connectivity between the Tab5 and the audio interface in the current tested hardware configuration.
- **M5Stack Tab5** — Captures, processes, and displays the digital I/Q data.

The USB audio interface connects to the Tab5 through a USB hub in the current tested hardware configuration. USB audio interface and hub compatibility may vary depending on the device and its USB implementation.

The application currently uses a stereo USB audio interface operating at 48 kHz. Compatibility depends on the USB audio device's supported formats and its ability to enumerate and operate with the ESP32-P4 USB host implementation.

## Spectrum Display

The panadapter displays the calculated spectrum across the available screen width.

The display includes:

- Real-time spectrum visualization
- Signal amplitude displayed in relative dBFS units
- Adjustable averaging to reduce noise and stabilize the display
- Baseline calculation and reset functionality
- Frequency markers and touch-based frequency selection
- Frequency-range adjustment through touch and drag gestures
- Display scaling and configuration controls

The spectrum is centered around the KX3's current receive frequency. Changes to the radio's tuning are reflected in the displayed frequency range.

## Waterfall Display

A scrolling waterfall display is positioned below the spectrum.

Each spectrum update contributes a horizontal line to the waterfall. Signal amplitude is represented by color intensity, allowing signals and frequency activity to be observed over time.

The waterfall provides a historical view of band activity, making it easier to identify intermittent signals, weak signals, frequency occupancy, and changes in propagation.

The waterfall scroll rate and display parameters can be adjusted through the application settings.

## Radio Frequency Synchronization

The application is designed to remain synchronized with the KX3's operating frequency.

Frequency changes made using the KX3's tuning controls or the Tab5's touch interface update the panadapter display accordingly.

Touch interaction with the spectrum allows the user to select frequencies and adjust the displayed frequency range.

## User Interface

The application provides an integrated interface for controlling and monitoring the panadapter.

The primary display includes:

- Spectrum display
- Waterfall display
- Current operating frequency
- Band and operating mode information
- Frequency selection and navigation
- Baseline reset control
- Settings panel

The settings panel provides access to display and signal-processing parameters, including averaging time and waterfall behavior.

## Hardware Requirements

### Required Hardware

- M5Stack Tab5
- Elecraft KX3 transceiver
- Elecraft KXUSB interface
- USB stereo audio interface
- Appropriate audio and USB cables

### USB Hub

A USB hub is required for the current tested hardware configuration to connect the USB audio interface to the Tab5.

Direct connection of compatible USB audio interfaces may be possible in other configurations, but has not been established as a general supported configuration.

The hub must be compatible with the Tab5 USB host implementation and support the USB audio interface without disrupting audio streaming.

### USB Audio Interface Requirements

The audio interface must support:

- Two simultaneous input channels
- Stereo audio capture
- 48 kHz sampling rate
- USB audio operation compatible with the Tab5 USB host implementation

A stereo input interface is required because the KX3 provides separate I and Q signals which allows image rejection. USB sound cards with only a mono microphone input are not suitable.

USB Audio Class compatibility is an important consideration when selecting an interface. Compatibility with Windows or macOS does not necessarily guarantee compatibility with the ESP32-P4 USB host.

## Software Architecture

The application is developed using:

- ESP-IDF
- ESP32-P4
- LVGL graphics library
- USB host audio functionality
- FFT-based signal processing

The software is organized around audio acquisition, signal processing, display rendering, and user interaction.

The audio acquisition subsystem receives stereo samples from the USB audio interface. The signal-processing subsystem performs FFT analysis and spectrum calculations. The display subsystem renders the panadapter and waterfall, while the user-interface subsystem handles touch input, configuration, and frequency navigation.

## Performance

The application is designed for continuous real-time spectrum monitoring.

Performance depends on the USB audio interface, USB hub, FFT configuration, display rendering workload, and graphics processing overhead.

The application includes performance instrumentation for monitoring loop execution time, spectrum update rate, and processing overhead. These measurements are used to optimize responsiveness and display refresh performance.

## Project Goals

The primary goals of the project are:

- Provide a portable panadapter for the Elecraft KX3.
- Use inexpensive and readily available USB audio hardware.
- Provide real-time spectrum and waterfall visualization.
- Enable intuitive frequency selection using the Tab5 touchscreen.
- Provide a configurable display and signal-processing interface.
- Develop a hardware and software configuration that can be reproduced by other amateur radio operators.

## Status

This project is under active development. Hardware compatibility, USB audio interface support, signal processing, user-interface functionality, and performance continue to evolve.

The repository contains the software, configuration, and development information necessary to build and experiment with the KX3 Panadapter.
