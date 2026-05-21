# How to Flight for the First Time
- To login into the lab computer, you need to plugin the Ethernet cable.
- Your username is your PID (your e-mail without the @vt.edu section)
- Your password is the same as your canvas pasword
- The computer might take long to configure your credentials, just wait.
- After your setup is done, disconnect the Ethernet cable, because this might cause conflicts with the wifi conection needed to link with the UAV.
- The remote controllers should always be charged, their batteries are compatible and their connectors are interchangable.

## On warehouse computer
- Run Vicon 3.10.0 
<!-- 7 (not 3.8, all the software stack has been tested on 3.7). -->
- Perform Calibration
    - Before calibration, hide the UAV, because the reflective balls might triger the calibration and cause it to perform bad
    - Conect the VICON cable to turn on the cameras
    - Wait for them to turn completely on (The lights of the cameras should turn blue and numbers should appear in the cameras, sometimes this takes longer)
    - Turn on the VICON wand, it always has to be used in continuous mode, never in strobe.
    - For calibration, you have to `
- Connect to the ordroid access point
- Run Vicon Stream cpp file "Public/ViconROS2/platform/viconStream.cpp"

## On my computer
- Connect the telemetry antenna
- Get the RC (remote control)
- QGroudControl
- Matlab
- CMD
- ssh odroid@192.168.12.1
- Pwd odroid
 - Here Run ./start_uxrce.sh
 - ./run_flightstack.sh