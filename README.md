# Real-time Sensing of Upper Extremity Movement Diversity Using Kurtosis Implemented on a Smartwatch

**Author:** Guillem Cornella i Barba  
**Affiliation:** Department of Mechanical and Aerospace Engineering, University of California, Irvine  
**Email:** cornellg@uci.edu  
**Date:** June 20, 2024  

## Overview

This repository contains Python code for validating a **Rolling Sample Kurtosis (RSK)** algorithm designed for **real-time sensing of upper extremity movement diversity** on a smartwatch.

The implementation simulates incoming sensor samples, maintains a circular buffer, and updates kurtosis incrementally using rolling moment equations rather than recomputing statistics over the full window each time. This approach is intended to reduce computational load and support **low-power, continuous wearable sensing**.

The code was developed in the context of movement monitoring for wearable neurorehabilitation applications.

## Purpose

The main goal of this script is to validate an efficient rolling kurtosis implementation that can be used for real-time movement diversity estimation from sensor streams.

In particular, this code:

- simulates sensor updates from pre-recorded data
- fills and updates a circular buffer
- computes kurtosis incrementally using rolling statistics
- compares the rolling approach with a full-buffer kurtosis calculation framework
- benchmarks runtime performance

## Key Features

- **Rolling Sample Kurtosis (RSK)** implementation
- **Incremental updates** while the buffer is filling
- **Rolling updates** once the buffer is full
- **Circular buffer simulation** for streaming sensor data
- **Runtime benchmarking** using `timeit`
- Optional framework for comparison against full-buffer computation and SciPy-based kurtosis

## File Structure

Expected files:

```text
.
├── rolling_kurtosis_validation.py
├── data_10000.json
└── README.md
