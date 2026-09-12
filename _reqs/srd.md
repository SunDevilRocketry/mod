<!-- next traceable tag: RQ.MOD.00035 -->
<!-- This req doc is written and updated manually and checked in along with source code -->
<!-- Desired behaviors are expected to be specified by human engineers. No AI assistance allowed for requirement specification -->
# SDR "mod" Library Software Requirements Document

### QA level: Mission Critical
### Part Number: A0013-XXX

## 1. Structural

RQ.MOD.00001 - The library shall be designed to interface with the Sun Devil Rocketry "driver" library.

    > Test plan: For the purposes of verifying the library, tests will assume these modules are used on A0002-2XX Flight Computer. This does not require the flight computer to release at the same time, however library integration is tested via the flight computer. Thus, the required coverage burden will only be achieved if the flight computer provides much of it.

RQ.MOD.00002 - The library shall be organized into self-contained modules with header files organized next to source files.

RQ.MOD.00003 - The library shall be designed to be compiled with GNU GCC for the desired target architecture.

## 2. Functional

### 2.1. Commands

RQ.MOD.00004 - The library shall provide a ping response code based on the provided compiler macro from the project.

RQ.MOD.00005 - The library shall provide a method for the flight computer to send a data package to the dashboard over USB.

RQ.MOD.00006 - The library shall provide a method for the flight computer to construct the dashboard data packet.

### 2.2. Debug

RQ.MOD.00007 - The library shall provide a platform-independent method to log debug data.

RQ.MOD.00008 - The debug logger shall use a circular buffer without dynamic memory allocation.

RQ.MOD.00009 - The debug logger shall accept a callback to be used to write debug data asynchronously.

RQ.MOD.00010 - The debug logger shall accept a callback to be used if the buffer overflows.

RQ.MOD.00011 - The debug logger shall provide a default implementation of the buffer overflow callback.

RQ.MOD.00012 - The debug logger shall provide a function to be called after an asynchronous write completes.

### 2.3. Error

RQ.MOD.00013 - The library shall provide an error handler.

RQ.MOD.00014 - If the USE_ERROR_CALLBACK macro is defined, the error handler shall search a lookup table for a callback provided by the project given an error code.

RQ.MOD.00015 - The error handler shall provide an overridable default error callback if the lookup fails or USE_ERROR_CALLBACK is not defined.

### 2.4. Math

RQ.MOD.00016 - The library shall provide a method to compute a Castagnoli 32-bit cyclic redundancy check.

The library shall provide all of the following quaternion operations:

    RQ.MOD.00017 - Hamilton Product (multiplication)

    RQ.MOD.00018 - Dot product

    RQ.MOD.00019 - Addition

    RQ.MOD.00020 - Scalar Multiplication

    RQ.MOD.00021 - Unit normalization

    RQ.MOD.00022 - Conjugation

### 2.5. Sensor

RQ.MOD.00023 - The library shall provide a method to start retrieval of sensor data

RQ.MOD.00024 - The library shall provide a method to retrieve sensor data from the last cycle and start retrieval of sensor data

The library shall provide a method to execute the following sensor commands and subcommands:

    RQ.MOD.00025 - Dump: Transmit the contents of the last sensor data cycle over USB.

RQ.MOD.00026 - The library shall provide methods to get and set the starting orientation of the flight computer.

The library shall provide the following IMU and magnetometer utilities:

    RQ.MOD.00027 - Conversion of raw IMU data to floating point values

    RQ.MOD.00028 - Integration of acceleration data to yield velocity

    RQ.MOD.00029 - Re-mapping raw IMU data based on orientation

    RQ.MOD.00030 - Fusion of raw IMU data to yield a unit quaternion orientation estimate

RQ.MOD.00031 - The library shall provide a method to convert a barometric pressure and temperature reading to an ISA altitude.

### 2.6. Telemetry

RQ.MOD.00032 - The library shall provide a method to determine the next message for the flight computer to send.

The library shall provide a method to construct each of the following messages:

    RQ.MOD.00033 - A packet containing vehicle position and orientation information

    RQ.MOD.00034 - A packet containing vehicle identification information

    RQ.MOD.00035 - A packet containing vehicle calibration information