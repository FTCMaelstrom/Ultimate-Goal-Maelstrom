package MidnightLibrary.MidnightMath;

import static java.lang.System.nanoTime;

/**
 * Created by Archishmaan Peyyety on 9/17/18.
 * Project: MasqLib
 */

public class MidnightIntegrator {
  private double prev = 0;
  private double prevTime = System.nanoTime();
  private double integral = 0;
  public double getIntegral (double current, double tChange) {
    integral += tChange * (0.5 * (current + prev));
    prev = current;
    prevTime = System.nanoTime()/1e9;
    return integral;
  }
  public double getIntegral (double current) {
    return getIntegral(current,System.nanoTime()/1e9 - prevTime);
  }

  public void reset() {
    prev = 0;
    prevTime = nanoTime();
    integral = 0;
  }
}