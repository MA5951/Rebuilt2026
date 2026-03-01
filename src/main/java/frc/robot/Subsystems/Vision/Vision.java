
package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;

import com.MAutils.Logger.MALog;
import com.MAutils.Vision.VisionSystem;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Util.Field;

public class Vision {
    private static Vision instance;
    private double lastTX;
    private double delta = 0;
    private int tagId = 0;

    double distance = 0;

    private Vision() {
        VisionSystem.getInstance()
                .setCameras(VisionConstants.FRONT_LL, VisionConstants.BACK_LL);

        lastTX = VisionConstants.FRONT_LL.getCameraIO().getTag().txnc;
    }

    public boolean isMainTag() {
        tagId = VisionConstants.FRONT_LL.getCameraIO().getTag().id;
        return tagId == 21 || tagId == 26 || tagId == 18 || tagId == 10 || tagId == 2 || tagId == 5;
    }

    public int getTagID() {
        return VisionConstants.FRONT_LL.getCameraIO().getTag().id;
    }

    public double getDistanceTryg() {
        distance = getRawTrigDistance();
        return distance;
    }

    public double getRawTrigDistance() {
        MALog.log("Subsystems/Vision/Roll", -VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch);
        return (Field.HUB_TAG_HIGHT - VisionConstants.FRONT_LL_HIGHT)
                / Math.tan(Math.toRadians((-VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch) + getCOrrectedTy()));

    }

    public double getFilteredTx() {
        return VisionConstants.FRONT_LL.getCameraIO().getTag().txnc > -0.8 &&
                VisionConstants.FRONT_LL.getCameraIO().getTag().txnc < 0 ? 0
                        : VisionConstants.FRONT_LL.getCameraIO().getTag().txnc;
    }

    public double getCOrrectedTy() {
        // VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch
        return VisionConstants.FRONT_LL.getCameraIO().getTag().tync
                * Math.cos(Math.toRadians(-VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch))
                - VisionConstants.FRONT_LL.getCameraIO().getTag().txnc
                        * Math.sin(Math.toRadians(-VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch));
    }

    public boolean isDeltaTx() {
        return Math.abs(getDeltaTX()) < 2.5;
    }

    public double getDeltaTX() {
        delta = VisionConstants.FRONT_LL.getCameraIO().getTag().txnc - lastTX;
        lastTX = VisionConstants.FRONT_LL.getCameraIO().getTag().txnc;
        return delta;
    }

    public void filterMainTag() {
        VisionConstants.FRONT_LL.getCameraIO().allowTags(Field.MAIN_TAGS);
    }

    public void filterCenterMainTag() {
        VisionConstants.FRONT_LL.getCameraIO().allowTags(Field.CENTER_MAIN_TAGS);
    }

    public void resetFilter() {
        VisionConstants.FRONT_LL.getCameraIO().allowTags(Field.ALL_TAGS);
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

}
