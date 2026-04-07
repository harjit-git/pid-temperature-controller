# PID Temperature Controller

**Course:** McMaster EP3BB3  
**Tools:** MSP430 (CCS), MATLAB App Designer, UART Serial  

## Overview
Closed-loop PID temperature controller capable of heating and cooling 
a small enclosed volume across 5–45°C with 0.1°C resolution.

## Files
- `Project_appD_v2_final.m` — MATLAB App Designer GUI (real-time plotting, PID tuning sliders)
- `firmware/` — MSP430 CCS project (ADC, PWM, UART communication)

## Features
- Real-time temperature plot with setpoint tracking
- Adjustable Kp, Ki, Kd via sliders
- Serial communication at 9600 baud
