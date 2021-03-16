package Library17822;

import Library17822.MidnightWrappers.MidnightController;
import Library17822.MinightResources.MidnightHelpers.MidnightHardware;

/**
 * Created by Archish on 2/12/18.
 */

public interface MidnightSubSystem {
    void DriverControl(MidnightController controller) throws InterruptedException;
    String getName();
    MidnightHardware[] getComponents();
}
