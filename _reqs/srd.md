<!-- next traceable tag: RQ.MOD.00053 -->
<!-- This req doc is written and updated manually and checked in along with source code -->
<!-- Desired behaviors are expected to be specified by human engineers. No AI assistance allowed for requirement specification -->
# SDR "mod" Library Software Requirements Document

### QA level: Safety Critical
### Part Number: A0015-XXX

## 1. Structural

RQ.MOD.00002 - The library shall be organized into self-contained modules with header files organized next to source files.

RQ.MOD.00003 - The library shall be designed to be compiled with GNU GCC for the desired target architecture.

## 2. Functional

### 2.1. Commands

RQ.MOD.00004 - The library shall provide a ping response code based on the provided compiler macro from the project.

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

RQ.MOD.00040 - The library shall provide a method to clamp a float within a specified range.

The library shall provide all of the following quaternion operations:

    RQ.MOD.00017 - Hamilton Product (multiplication)

    RQ.MOD.00018 - Dot product

    RQ.MOD.00019 - Addition

    RQ.MOD.00020 - Scalar Multiplication

    RQ.MOD.00021 - Unit normalization

    RQ.MOD.00022 - Conjugation

    RQ.MOD.00047 - Creation from euler angles

    RQ.MOD.00048 - Determine whether quat is finite

    RQ.MOD.00049 - Body-to-world frame transition

    RQ.MOD.00050 - World-to-body frame transition

The library shall provide all of the following 3-element vector operations:

    RQ.MOD.00041 - Addition

    RQ.MOD.00042 - Cross Product

    RQ.MOD.00043 - Determine whether vector is finite

    RQ.MOD.00044 - Determine magnitude

    RQ.MOD.00045 - Normalization

    RQ.MOD.00046 - Scalar Multiplication

### 2.5. Telemetry

RQ.MOD.00051 - The library shall control Sun Devil Rocketry's one-way telemetry interface by defining the authoritative version in a header for projects to integrate.

### 2.6. Nav

RQ.MOD.00053 - The library shall provide a method to fuse sensor data.

RQ.MOD.00054 - The library shall provide a method to initialize the sensor fusion system.

RQ.MOD.00055 - The library shall call a contract function to prevent concurrent access to its state before accessing state data.

RQ.MOD.00056 - The library shall invalidate output data that cannot be determined from the configured sources.

RQ.MOD.00057 - The library shall provide a method to reset velocity integration.

RQ.MOD.00064 - The library shall provide a valid default configuration.

#### 2.6.1. Mahony

RQ.MOD.00036 - The library shall provide a Mahony filter to fuse accelerometer and gyroscope data to return an attitude estimate.

RQ.MOD.00037 - The library shall report vehicle orientation with a unit quaternion.

The library shall disable accelerometer fusion in the following cases:

    RQ.MOD.00038 - The project disables fusion.

    RQ.MOD.00039 - The provided parameters for accelerometer fusion are invalid.

#### 2.6.2. ISA

RQ.MOD.00031 - The library shall provide a method to convert a barometric pressure and temperature reading to an ISA altitude.

RQ.MOD.00051 - The library shall provide a method to retrieve a ground reference elevation from current and calibrated pressure and temperature.

RQ.MOD.00052 - The library shall provide a method to retrieve the ISA temperature given a barometric pressure reading.

#### 2.6.3. POF

RQ.MOD.00058 - The library shall provide a method to set the phase of flight.

RQ.MOD.00059 - The library shall provide a method to get the phase of flight.

RQ.MOD.00060 - The library shall update phase of flight during sensor fusion.

The library shall be able to detect the following phase of flight transitions.

    RQ.MOD.00061 - On-Ground -> Powered Ascent

    RQ.MOD.00062 - Powered Ascent -> Unpowered Ascent

    RQ.MOD.00063 - Ascent -> Descent

    RQ<POSTPONED>.MOD.00065 - Descent -> Landed
