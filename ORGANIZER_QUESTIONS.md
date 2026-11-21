# Questions for the pilot and the organizers at first contact

Merged on 2026-11-21 from Aydan's list (`ORGANIZER_QUESTIONS.md` on the branch `origin/spec-justice-d`) and the audit of 2026-11-21. The order is by how much the answer changes our day. Stage 2 can answer some. Strike those out and write the answer next to the question.

1. **Firmware and link.** Which firmware and build is on our flight controller (ArduCopter, PX4, Betaflight or INAV, and its version)? Which route goes from the APB to the flight controller (serial port and baud rate)? Which MAVLink streams and rates may we ask for?
2. **The switch and the parameters.** Which flight mode does each position of the hold switch select? May our program write flight controller parameters (the outside-position source and the GPS type)? Who sets the failsafe actions (EKF failsafe, radio failsafe), and to which values? Will the crew turn off the vision arming check? In our simulator the flight controller does not arm with it on (`EVENT_DAY.txt` step 1, item 4a).
3. **The APB image.** Which versions of Python, numpy and OpenCV are on it? May we install wheels, or run our own Python from a folder? Is there an SD card for logs?
4. **The Justice D.** Which device node, pixel format and frame rate? Does a frame's time stamp come from the driver or from the read time? Does the pilot video run on the APB during the hold?
5. **Height.** Is a rangefinder fitted? May our program use its value? If not, the height is the barometer.
6. **The scored hold.** How long is it, and from which moment? Which drift limits apply (RMS, largest distance, height)? Which altitude levels are approved, and how many attempts for each level? How do the judges measure the position?
7. **GNSS and heading.** Which heading source does the build use with GNSS off, and is the compass calibrated? GNSS is permitted before the scored hold: which proof do the judges want that GNSS did not aid the hold?
8. **Time with the aircraft.** How much bench time with propellers off do we get before the first flight? We need about 10 minutes of it with the pilot, for the takeover demonstration (`PILOT_SAFETY.txt`). How do we book flights, and when is the first one?
9. **Conditions.** What is the wind limit? What is the ground surface at each level? May the pilot yaw after the switch?
10. **Evidence.** Which common recordings or reference tests score the estimation and the timing bonus, and in which file format?
11. **Other services.** How much CPU and memory do the other aircraft services use on the APB?

Not kept from the first list: the IMU part number. Our program uses the attitude stream of the flight controller, so the answer does not change our day.
