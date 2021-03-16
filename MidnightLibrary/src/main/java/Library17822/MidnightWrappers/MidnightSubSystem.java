package Library17822.MidnightWrappers;

import Library17822.MidnightWrappers.MidnightController;
import Library17822.MidnightResources.MidnightHelpers.MidnightHardware;

/**
 * Created by Archish on 2/12/18.
 */
public interface MidnightSubSystem {
    void DriverControl(MidnightController controller) throws InterruptedException;
    String getName();
    MidnightHardware[] getComponents();
}
