
package frc.robot.Subsystems.Vision;

import com.MAutils.Vision.VisionSystem;

import frc.robot.Util.Field;

public class Vision {
    private static Vision instance;

    
    private Vision() {
        VisionSystem.getInstance()
                .setCameras(VisionConstants.FRONT_LL, VisionConstants.BACK_LL);
    }

    public boolean isMainTag() {
        return VisionConstants.FRONT_LL.getCameraIO().getTag().id == Field.getMainTagID();
    }

    public double getDistanceTryg() {
        return 0; //TODO can imploment
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
    
}
