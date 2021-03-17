package Library17822.MidnightResources;

import Library17822.MidnightResources.MidnightController;
import Library17822.MidnightResources.MidnightHardware;

/**
 * Created by Archish on 2/12/18.
 */

public interface MidnightSubSystem {
    void DriverControl(MidnightController controller) throws InterruptedException;
    String getName();
    MidnightHardware[] getComponents();
}
