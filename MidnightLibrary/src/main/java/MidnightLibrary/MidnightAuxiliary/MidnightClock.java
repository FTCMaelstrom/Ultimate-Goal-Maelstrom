package MidnightLibrary.MidnightAuxiliary;

import java.util.Locale;

import static MidnightLibrary.MidnightAuxiliary.MidnightClock.Resolution.SECONDS;

public class MidnightClock implements MidnightHardware {

    private long startTime;

    private boolean isPaused = false;
    private long pauseStart;
    private long pauseLength;

    private final String name = "CLOCK";


    public MidnightClock() {
        this.reset();
    }

    public void reset() {
        isPaused = false;
        pauseStart = 0L;
        pauseLength = 0L;
        startTime = System.nanoTime();
    }

    public long nanoseconds() {
        return System.nanoTime() - startTime - pauseLength;
    }

    public double milliseconds() {
        return nanoseconds() * 1E-6;
    }

    public double seconds() {
        return nanoseconds() * 1E-9;
    }

    public boolean elapsedTime(double time, Resolution resolution) {
        return nanoseconds() > (long) (time * resolution.multiplier);
    }

    public void pause() {
        if (!isPaused) pauseStart = System.nanoTime();
        isPaused = true;
    }

    public void resume() {
        if (isPaused) pauseLength += (System.nanoTime() - pauseStart);
        isPaused = false;
    }

    public boolean isPaused() {
        return isPaused;
    }

    public String getName() {
        return name;
    }

    public String[] getDash() {
        return new String[]{
                String.format(Locale.US, "%.5f  %s", seconds(), isPaused() ? "PAUSED" : "")
        };
    }

    public boolean hasNotPassed(double time, Resolution resolution) {
        return nanoseconds() < (long) (time * resolution.multiplier);
    }

    public boolean hasNotPassed(double time) {
        return hasNotPassed(time, SECONDS);
    }

    public enum Resolution {
        NANOSECONDS(1),
        MILLISECONDS(1E6),
        SECONDS(1E9);
        public final double multiplier;

        Resolution(double multiplier) {
            this.multiplier = multiplier;
        }
    }

}