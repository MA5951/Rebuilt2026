
package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;

import com.MAutils.Logger.MALog;
import com.MAutils.Vision.VisionSystem;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.Swerve.Swerve;
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
        MALog.log("Subsystems/Vision/Roll",-VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch);
        return (Field.HUB_TAG_HIGHT - VisionConstants.FRONT_LL_HIGHT)
                / Math.tan(Math.toRadians((-VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch) + getCOrrectedTy()));
        // ARI'S SHIT
        // double pitch = VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch;
        // double yaw = Swerve.getInstance().getGyroData().yaw;
        // Rotation3d Rpitch = new Rotation3d(Math.toRadians(pitch), 0, 0);
        // Rotation3d Ryaw = new Rotation3d(0, 0, Math.toRadians(yaw-90));
        // Matrix<N3, N3> R = Rpitch.rotateBy(Ryaw).toMatrix();
        // Matrix<N3, N1> vc = new Matrix<N3,N1>(Nat.N3(), Nat.N1(), new double[] {
        //     Math.tan(Math.toRadians(VisionConstants.FRONT_LL.getCameraIO().getTag().txnc)),
        //     Math.tan(Math.toRadians(VisionConstants.FRONT_LL.getCameraIO().getTag().tync)),
        //     1});
        // Matrix<N3, N1> vw = R.times(vc);
        // return vw.get(2, 0);
    }

    public double getCOrrectedTy() {
        //VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch
        return VisionConstants.FRONT_LL.getCameraIO().getTag().tync * Math.cos(Math.toRadians(-VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch))- VisionConstants.FRONT_LL.getCameraIO().getTag().txnc * Math.sin(Math.toRadians(-VisionConstants.FRONT_LL.getCameraIO().getIMU().Pitch));
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

}
