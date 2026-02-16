
package frc.robot.Util;

import com.MAutils.Utils.DriverStationUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class Field {
    
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    
    public static final double LENGTH = TAG_LAYOUT.getFieldLength();
    public static final double WIDTH = TAG_LAYOUT.getFieldWidth();

    public static final int BLUE_MAIN_TAG = 26;
    public static final int RED_MAIN_TAG = 10;
    public static final double HUB_TAG_HIGHT = Units.inchesToMeters(44.25);

    public static final double ALLIANCE_WIDTH = TAG_LAYOUT.getTagPose(26).get().getX();
    public static final Translation2d MIDDLE = new Translation2d(LENGTH / 2, WIDTH / 2);
    public static final Translation2d FIELD_CORNER = new Translation2d(LENGTH, WIDTH);
    public static final double HUB_WIDTH = Units.inchesToMeters(47);
    public static final Translation2d BLUE_HUB = new Translation2d(ALLIANCE_WIDTH + HUB_WIDTH / 2, MIDDLE.getY());
    public static final Translation2d RED_HUB = new Translation2d((LENGTH - ALLIANCE_WIDTH) - HUB_WIDTH / 2, MIDDLE.getY());
    public static final Translation2d BLUE_ALLIANCE_CORNER = new Translation2d(ALLIANCE_WIDTH, WIDTH);
    public static final Translation2d RED_ALLIANCE_CORNER = new Translation2d(LENGTH - ALLIANCE_WIDTH, WIDTH);
    public static final double NET_LENGTH = Units.inchesToMeters(58.41);
    public static final double NET_X_OFFSET = Units.inchesToMeters(10.26);
    public static final int[] MAIN_TAGS = {BLUE_MAIN_TAG, RED_MAIN_TAG};
    public static final int[] ALL_TAGS = {1, 2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

    public static final Rectangle2d FIELD_RECTANGLE = new Rectangle2d(new Translation2d(0,0), FIELD_CORNER);

    public static final Translation2d BLUE_NET_A = new Translation2d(ALLIANCE_WIDTH + NET_X_OFFSET, (WIDTH / 2) - (NET_LENGTH / 2));
    public static final Translation2d BLUE_NET_B = new Translation2d(ALLIANCE_WIDTH + NET_X_OFFSET, (WIDTH / 2) + (NET_LENGTH / 2));

    public static final Translation2d RED_NET_A = new Translation2d((LENGTH - ALLIANCE_WIDTH) - NET_X_OFFSET, (WIDTH / 2) - (NET_LENGTH / 2));
    public static final Translation2d RED_NET_B = new Translation2d((LENGTH - ALLIANCE_WIDTH) - NET_X_OFFSET, (WIDTH / 2) + (NET_LENGTH / 2));


    public static Translation2d getHub() {
        return DriverStationUtil.getAlliance() == DriverStation.Alliance.Blue ? BLUE_HUB : RED_HUB;
    }

    public static int getMainTagID() {
        return DriverStationUtil.getAlliance() == DriverStation.Alliance.Blue ? BLUE_MAIN_TAG : RED_MAIN_TAG;
    }

    public static Translation2d getNetA() {
        return DriverStationUtil.getAlliance() == DriverStation.Alliance.Blue ? BLUE_NET_A : RED_NET_A;
    }

    public static Translation2d getNetB() {
        return DriverStationUtil.getAlliance() == DriverStation.Alliance.Blue ? BLUE_NET_B : RED_NET_B;
    }

    public static double getAllianceXLine() {
        return DriverStationUtil.getAlliance() == DriverStation.Alliance.Blue ? ALLIANCE_WIDTH : LENGTH - ALLIANCE_WIDTH;
    }   

}
