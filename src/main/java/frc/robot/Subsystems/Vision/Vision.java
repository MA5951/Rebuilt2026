
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
        return (Field.HUB_TAG_HIGHT - VisionConstants.FRONT_LL_HIGHT)
                / Math.tan(Math.toRadians(VisionConstants.FRONTLL_ANGLE + VisionConstants.FRONT_LL.getCameraIO().getTag().tync)); //TODO why dont use the limlight internal gyro
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

}
