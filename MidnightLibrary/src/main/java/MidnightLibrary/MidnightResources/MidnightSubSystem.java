package MidnightLibrary.MidnightResources;

/**
 * Created by Archish on 2/12/18.
 */

public interface MidnightSubSystem {
    void DriverControl(MidnightController controller) throws InterruptedException;
    String getName();
    MidnightHardware[] getComponents();
}
