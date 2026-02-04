
package frc.robot.Subsystems.Swerve;

import com.MAutils.Swerve.Controllers.AngleAdjustController;
import com.MAutils.Swerve.Controllers.FieldCentricDrive;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.SwerveSystemConstants.GearRatio;
import com.MAutils.Swerve.SwerveSystemConstants.WheelType;
import com.MAutils.Swerve.Utils.PIDController;
import com.MAutils.Swerve.Utils.SwerveState;
import com.MAutils.Utils.GainConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.PortMap;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Vision.VisionConstants;

public class SwerveConstants {

        public static final GainConfig driveGainConfig = new GainConfig().withKV(0.765).withKS(0.23).withKP(0.5);
        public static final GainConfig turnGainConfig = new GainConfig().withKP(150).withKS(0.23);

        // Swerve System Constants
        public static final SwerveSystemConstants SWERVE_CONSTANTS = new SwerveSystemConstants()
                        .withPyshicalParameters(0.6, 0.6, 52, WheelType.BLACK_TREAD, 6.25)
                        .withMotors(DCMotor.getKrakenX60(1), DCMotor.getFalcon500(1),
                                        PortMap.SwervePorts.SWERVE_MODULE_IDS,
                                        PortMap.SwervePorts.PIGEON2)
                        .withMaxVelocityMaxAcceleration(4.9, 10)
                        .withOdometryUpdateRate(250)
                        .withDriveCurrentLimit(80, true)// 45
                        .withTurningCurrentLimit(50, false).withDriveTuning(driveGainConfig)
                        .withTurningTuning(turnGainConfig)
                        .withGearRatio(GearRatio.L2);

        // PID Controllers
        public static final PIDController ABS_PID_CONTROLLER = new PIDController(0.09, 0, 0)
                        .withContinuesInput(-180, 180)
                        .withTolerance(3);

        public static final PIDController REL_PID_CONTROLLER = new PIDController(0.09, 0, 0)
                        .withContinuesInput(-180, 180)
                        .withTolerance(3);

       
        // Swerve Drive Controllers
        public static final FieldCentricDrive FIELD_CENTRIC_DRIVE = new FieldCentricDrive(
                        RobotContainer.getDriverController(), SWERVE_CONSTANTS);

        public static final AngleAdjustController ANGLE_ADJUST_CONTROLLER = new AngleAdjustController(SWERVE_CONSTANTS,
                        ABS_PID_CONTROLLER);

        // Swerve States
        public static final SwerveState NONE = new SwerveState("NONE").withXY(0, 0).withOmega(0);

        public static final SwerveState FIELD_CENTRIC = new SwerveState("Field Centric")
                        .withOnStateEnter(() -> FIELD_CENTRIC_DRIVE.withSclers(1, 0.7))
                        .withSpeeds(FIELD_CENTRIC_DRIVE);

        public static final SwerveState FIELD_CENTRIC_40 = new SwerveState("Field Centric 40 Precent")
                        .withOnStateEnter(() -> FIELD_CENTRIC_DRIVE.withSclers(0.8, 0.8))
                        .withSpeeds(FIELD_CENTRIC_DRIVE);

        public static final SwerveState SHOOTING_ABS = new SwerveState("Shooting Absolute")
                        .withOnStateEnter(() -> {
                                ANGLE_ADJUST_CONTROLLER.withPIDController(ABS_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withSetPoint(() -> SuperStructure.getAbsAngleToTarget()); //TODO we need to talk about when we enter tha shooting state 
                                ANGLE_ADJUST_CONTROLLER.withGyroSupplier(Swerve.getInstance().getAbsYawSupplier()); 
                        })
                        .withSpeeds(ANGLE_ADJUST_CONTROLLER);


        public static final SwerveState SHOOTING_REL = new SwerveState("Shooting Relative")
                        .withOnStateEnter(() -> {
                                ANGLE_ADJUST_CONTROLLER.withPIDController(REL_PID_CONTROLLER); //TODO not the right pid
                                ANGLE_ADJUST_CONTROLLER.withSetPoint(() -> SuperStructure.getRelAngleToTarget());
                                ANGLE_ADJUST_CONTROLLER.withGyroSupplier(Swerve.getInstance().getGyroYawSupplier()); 
                        })
                        .withSpeeds(ANGLE_ADJUST_CONTROLLER);


        public static final SwerveState FEEDING = new SwerveState("Feeding")
                        .withOnStateEnter(() -> {
                                ANGLE_ADJUST_CONTROLLER.withPIDController(ABS_PID_CONTROLLER);
                                ANGLE_ADJUST_CONTROLLER.withSetPoint(() -> SuperStructure.getAngleToFeeding());
                                ANGLE_ADJUST_CONTROLLER.withGyroSupplier(Swerve.getInstance().getAbsYawSupplier()); 
                        })
                        .withSpeeds(ANGLE_ADJUST_CONTROLLER);

        public static final SwerveState FEEDING_IN_MOTION = new SwerveState("Feeding In Motion")
                        .withOnStateEnter(() -> FIELD_CENTRIC_DRIVE.withSclers(0.6, 0.4))
                        .withSpeeds(FIELD_CENTRIC_DRIVE);

}