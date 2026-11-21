# Questions for the organizers (send through the event email thread; answers go to all teams)

Open after the Stage 1 package (rev 1.3). Stage 2 may answer some; strike those out.

1. How is the Justice D exposed on the APB (device node, GStreamer or libcamera pipeline, pixel format)? Is there a supplied acquisition example?
2. Are frame time stamps from the sensor/driver, and how do they relate to the IMU/FC clock (timestamp convention, camera-to-IMU delay)?
3. Is any rangefinder supplied, or is height only the barometer (DPS310) and the FC's estimate?
4. Which firmware builds are issued (ArduPilot, PX4, Betaflight, INAV), and which MAVLink interface and rates do they expose to the APB?
5. What software is on the APB image (Python, numpy, OpenCV with V4L2/GStreamer)? How do we deploy and run our program?
6. What are the approved altitude levels, the qualifying-hold definition (radius, duration, drift limit) and the measurement method?
7. May the pilot yaw or move after engaging the hold? Roughly what wind and ground texture should we expect at each altitude?
8. How are the pilot's engage and disengage switches mapped (which FC mode)?
9. What CPU and memory do the other aircraft services use on the APB (area C: competition for resources)?
10. Is the ICM42605 (spec) the IMU on the event aircraft, or the ICM-42688 that community sources list?
11. What practice data, recordings or simulation material will be shared, and how are development flights booked?
