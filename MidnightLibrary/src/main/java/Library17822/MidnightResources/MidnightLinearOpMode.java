package Library17822.MidnightResources;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

import Library17822.MidnightResources.MidnightClock;
import Library17822.MidnightResources.MidnightController;
import Library17822.MidnightResources.MidnightDashBoard;
import Library17822.MidnightResources.MidnightUtils;


/**
 * Custom Linear opMode
 */

public abstract class MidnightLinearOpMode extends LinearOpMode {
    protected MidnightDashBoard dash;
    protected MidnightController controller1, controller2;
    protected MidnightClock timeoutClock = new MidnightClock();
    public final void runOpMode() throws InterruptedException {
        try {
            dash = new MidnightDashBoard(super.telemetry);
            dash.setNewFirst();
            MidnightUtils.setLinearOpMode(this);
            controller1 = new MidnightController(super.gamepad1, "controller1");
            controller2 = new MidnightController(super.gamepad2, "controller2");
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }
    public abstract void runLinearOpMode() throws InterruptedException;
    public void stopLinearOpMode() {}
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
            for(Thread t : threads) if (!t.isAlive()) count++;
        }
    }
    public void sleep(double timeSeconds) {
        try {
            Thread.sleep((long) timeSeconds * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep() {sleep(1.);}
    public Gamepad getDefaultController() {
        return gamepad1;
    }
}