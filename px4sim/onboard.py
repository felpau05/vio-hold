"""The onboard program in its own process, as on the APB: apb_main.run_live with fc_link to the PX4 SITL
onboard port and a capture source that reads the simulator's rendered camera. Only the frame source and
the slow-CPU factor are simulator parts; everything from the frame to VISION_POSITION_ESTIMATE is the
code that flies.

Started by sitl.run_case (variant "apb") with multiprocessing (spawn):
  main(settings_path, frames_conn, stop, log_path, opts, status_q, clk)
opts: cpu_factor (slow board), reanchor_on_lost (None = the program's default, else an A/B test).
frames_conn delivers (picture as Hold gets it, t_grab) or None at the end. clk.value = wall
time.monotonic minus sim time (sitl.publish_clock): this process's time.monotonic runs on the sim clock,
because PX4's lockstep clock wobbles against wall time while a real FC and the APB agree within ppm.
"""
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)


def main(settings_path, conn, stop, log_path, opts, status_q, clk):
    real, last = time.monotonic, [-float("inf")]

    def sim_clock():
        v = max(real() - clk.value, last[0])      # never backward (clk moves in 10 ms steps)
        last[0] = v
        return v
    time.monotonic = sim_clock                    # every module reads time.monotonic at call time
    time.monotonic_ns = lambda: int(sim_clock() * 1e9)
    sys.path.insert(0, PROJ)
    import apb_main
    import capture
    import fc_link
    import hold
    import settings as S

    say = lambda kind, x: status_q.put((time.monotonic(), kind, x))   # noqa: E731
    import faulthandler                                # a crash or a hang leaves its stack here
    fault = open(log_path[:-4] + "_fault.txt", "w")
    faulthandler.enable(file=fault, all_threads=True)
    try:
        s = S.load(settings_path)
        cpu_factor = opts["cpu_factor"]
        if opts.get("reanchor_on_lost") is not None:
            apb_main.REANCHOR_ON_LOST = opts["reanchor_on_lost"]
        if cpu_factor > 1:
            # Simulator part: a board CPU cpu_factor times slower than this laptop. The real Hold runs,
            # then the thread waits (cpu_factor - 1) x its own time, so the frame age and the frames
            # skipped are those of the slow board.
            orig = hold.Hold.process

            def slow(self, *a):
                faulthandler.dump_traceback_later(0.3, file=fault)    # diagnostics: where a slow call is
                t1 = time.perf_counter()
                r = orig(self, *a)
                dt = time.perf_counter() - t1
                faulthandler.cancel_dump_traceback_later()
                if dt > 0.3:
                    say("slow", f"Hold.process took {dt:.2f} s ({r.state}, {r.why})")
                time.sleep((cpu_factor - 1) * dt)
                return r
            hold.Hold.process = slow

        import threading
        waits, latest, cv = [], [None, 0, 0], threading.Condition()

        def reader():                 # as V4L2Source: a thread reads every frame, newest() takes the latest
            while True:
                try:
                    item = conn.recv()
                except (EOFError, OSError):
                    item = None
                with cv:
                    latest[0], latest[1] = item, latest[1] + 1
                    cv.notify_all()
                if item is None:
                    return
        threading.Thread(target=reader, daemon=True).start()

        def newest():
            with cv:
                if not cv.wait_for(lambda: latest[1] > latest[2], 0.5):
                    return None
                item, latest[2] = latest[0], latest[1]
            if item is not None:
                waits.append(time.monotonic() - item[1])  # simulator check: frame age when the program takes it
                prog[0] = real()
                faulthandler.dump_traceback_later(3.0, file=fault)   # C-level timer: works with the GIL held
            return item
        src = capture.CallableSource(newest, s)
        if s["camera"] == "digital":
            # Simulator part: the drone's DigitalSource judges pictures with the digital checks (capture.judge);
            # CallableSource always takes the analog _Judge, whose repeat_mad of 1 grey refuses fresh low-noise frames.
            src.judge = capture.judge(s)
        fc = apb_main.open_fc(s)
        say("connected", f"{fc.firmware}, clock ready {fc.clock.ready()}, ATTITUDE {fc.rate('ATTITUDE', 1.0):.0f} Hz")

        def clock_report():           # simulator check: fc_link's FC-time -> monotonic mapping, twice a second
            while not stop.is_set():
                t_fc = fc.clock.last
                m = fc.clock.to_local(t_fc)
                if m is not None:
                    say("clock", (t_fc, m))
                time.sleep(0.5)
        threading.Thread(target=clock_report, daemon=True).start()
        prog = [real()]                                # wall time of the last frame taken (watchdog)

        def watchdog():
            dumped = False
            while not stop.is_set():
                if real() - prog[0] > 3.0 and not dumped:
                    faulthandler.dump_traceback(file=fault, all_threads=True)
                    fault.flush()
                    say("hang", f"no frame taken for {real() - prog[0]:.1f} s; stacks in {fault.name}")
                    dumped = True
                time.sleep(0.5)
        threading.Thread(target=watchdog, daemon=True).start()
        for ok, line in fc_link.check_params(fc, s):
            say("param", line)
        k = 0
        while not stop.is_set():          # as apb_main.mode_run: a failed start is retried
            try:
                apb_main.run_live(s, src, fc, stop=stop, log_path=log_path if k == 0 else f"{log_path[:-4]}_{k}.csv")
            except RuntimeError as e:
                say("run", f"FAIL {e}")
                if "will not start" in str(e):
                    break
            k += 1
            time.sleep(0.2)
        faulthandler.cancel_dump_traceback_later()
        w = sorted(waits[-1500:]) or [float("nan")]
        say("done", f"stopped after {k} start(s); frame age when taken p50 {1e3 * w[len(w) // 2]:.0f} ms, "
                    f"p95 {1e3 * w[int(len(w) * 0.95)]:.0f} ms; SCALED_IMU {fc.rate('SCALED_IMU', 2.0):.0f} Hz, "
                    f"{len(fc.imu.rows)} rows kept")
        fc.close()
    except Exception:
        say("error", traceback.format_exc())
