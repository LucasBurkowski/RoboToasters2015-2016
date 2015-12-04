package org.robotoasters.library.interfaces;

import com.qualcomm.robotcore.hardware.*;
/**
 * Created by Lucas on 12/1/2015.
 */
public class LSM303D {
    I2cDevice compass;
    I2cDeviceReader finder;
    public void Compass(){
        byte CURRENT_ADDRESS = 0x1D;
        int currentAddress = CURRENT_ADDRESS;
        compass.enableI2cReadMode(currentAddress, );
        compass.
    }
}
