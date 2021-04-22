package MidnightLibrary.MidnightAuxiliary;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.DEFAULT_SLEEP_TIME;
import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.setLinearOpMode;


/**
 * Custom Linear opMode
 */

/*
 * Modified 4/20/21 9:06 PM by Amogh Mehta
 */

public abstract class MidnightLinearOpMode extends LinearOpMode {
    public MidnightDashBoard dash;
    protected MidnightClock timeoutClock = new MidnightClock();

    public final void runOpMode() {
        try {
            dash = new MidnightDashBoard(super.telemetry);
            dash.setNewFirst();
            setLinearOpMode(this);
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }

    public abstract void runLinearOpMode();

    public void stopLinearOpMode() {
    }

    public void runSimultaneously(Runnable... runnables) {
        List<Thread> threads = new ArrayList<>();
        int i = 0;
        for (Runnable runnable : runnables) {
            threads.add(new Thread(runnable));
            threads.get(i).start();
            i++;
        }
        int count = 0;
        while (opModeIsActive() && count < i) {
            count = 0;
            for (Thread t : threads) if (!t.isAlive()) count++;
        }
    }

    public void sleep() {
        sleep(DEFAULT_SLEEP_TIME);
    }

    public Gamepad getDefaultController() {
        return gamepad1;
    }
}