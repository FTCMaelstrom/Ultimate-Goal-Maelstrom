package MidnightLibrary.MidnightResources;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Archish on 2/12/18.
 */

public interface MidnightSubSystem {
    void driverControl(Gamepad controller) ;
    String getName();
    MidnightHardware[] getComponents();
}
