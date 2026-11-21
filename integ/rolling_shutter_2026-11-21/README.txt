ROLLING SHUTTER AND CAMERA LOSS: MEASURED ON THE EVENT AIRCRAFT, 2026-11-21

Two recordings from the event aircraft, made with apb_rec.py to /tmp on the APB, then copied to the laptop.
The raw recordings are not in git (73 to 157 MB for each chunk). They are on Paul's laptop in ~/vio_board_rec.
The board clock is wrong. The laptop time of the recordings is about 17:46.


1. ROLLING SHUTTER (rs_ceiling_161057, 4 chunks)

Recording: 35 s, 1050 frames at 30 fps, props off. The aircraft is upside down, so the Justice D sees the
ceiling. The hand turns the aircraft quickly. Largest rates: 292 deg/s (gyro x), 103 deg/s (y), 135 deg/s (z).

Method (rs_readout.py): split the picture into 6 bands of rows. For each band, find the gyro time shift that
best predicts the image motion of that band. The fit uses only the central columns 280 to 680. It uses only
frame pairs with more than 1 deg of rotation and with all 6 bands measured: 199 pairs.
Run: python rs_readout.py ~/vio_board_rec/rs_ceiling_161057 readout.json

Result (readout.json):
  Band centre row (of 540):     45    135    225    315    405    495
  Time after the top band (ms):  0.0   +7.6  +12.4  +16.4  +19.4  +19.2
  - The direction is clear: the camera exposes the top rows first and the bottom rows last.
  - Top band to bottom band: 19 ms.
  - Line fit, first row to last row: 23.2 ms. Bootstrap 90 %: 19.7 to 24.8 ms.
  - First half of the recording: 24.3 ms. Second half: 12.8 ms. The halves do not agree.
  - At 60 fps a readout longer than 16.7 ms is not possible if the sensor reads one frame for each output frame.
    The method or the camera pipeline has an effect that we did not find. So we give no exact readout value.
  - settings.json readout_s (0.014) is not changed.

Picture: rolling_shutter_rows.png (left: one frame at 98 deg/s with the 6 bands, right: the band times).


2. CAMERA LOSS (rec_loop_161331, 6 chunks)

Recording: 60 s at 15 fps. 0 to 17 s: carried by hand with fast turns (40 to 107 deg/s). A hand covers the lens
at 11.2 s for about 0.5 s. 18 to 55 s: the aircraft lies still on a dark surface. This recording has no full
loop back to the start mark. It is not evidence for the onboard recovery bonus (Stage 2, section 07).

Result: detail score (fine_detail in apb_live.py), limit COVER_FINE = 3.0.
  - Normal frames while carried with fast turns: 3.3 to 3.8.
  - Hand over the lens: 2.8 to 3.0. The hand is bright and blurry, not dark.
  - The desk check (HARDWARE_BRINGUP.md row 9, aircraft held still): normal 16 to 19, hand 1.3 to 1.7.
  - Finding: while the aircraft moves fast, the margin between a normal frame and a cover is small.
    Motion blur lowers the score of a normal frame to near the limit.

Picture: camera_loss_detail.png (three frames: before, hand cover, after, and the detail score against time).
