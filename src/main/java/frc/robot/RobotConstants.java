
package frc.robot;

import com.MAutils.Auto.AutoRoutine;
import com.MAutils.AutoChooser.Autonomus;
import com.MAutils.RobotControl.MRobotState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.ClimbCommand;
import frc.robot.Commands.SandwichCommand;
import frc.robot.Commands.SwerveController;
import frc.robot.Commands.Auto.DepotClimb;
import frc.robot.Commands.Auto.FeedingAuto;
import frc.robot.Commands.Auto.FeedingBack;
import frc.robot.Commands.Auto.FeedingShooting;
import frc.robot.Commands.Auto.MagazineClimb;
import frc.robot.Commands.Auto.MgazineClimbRight;
import frc.robot.Commands.Auto.TwoMagazine;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Subsystems.Hood.HoodConstants;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.Kicker.KickerConstants;
import frc.robot.Subsystems.Roller.RollerConstants;
import frc.robot.Subsystems.Sandwich.Sandwich;
import frc.robot.Subsystems.Sandwich.SandwichConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Swerve.SwerveAutoFollower;
import frc.robot.Subsystems.Transfer.TransferConstants;
import frc.robot.Subsystems.Vision.Vision;

public class RobotConstants {

        public static final Autonomus NONE_AUTO = new Autonomus("None Auto", new InstantCommand(), new Pose2d(), 0);
        public static final Autonomus DEPOT_CLIMB = new Autonomus("Depot Climb", new DepotClimb(), SwerveAutoFollower.getStartPose("DC 1 Push"), -90);
        public static final Autonomus TWO_MAGAZINE = new Autonomus("Two Magazine", new TwoMagazine(), SwerveAutoFollower.getStartPose("TM5"), 0);
        public static final Autonomus FEEDING_AUTO = new Autonomus("Feeding Auto", new FeedingAuto(), SwerveAutoFollower.getStartPose("F1"), 180);
        public static final Autonomus FEEDING_BACK = new Autonomus("Feeding Back Auto", new FeedingBack(), SwerveAutoFollower.getStartPose("FB1"), 180);
        public static final Autonomus FEEDING_CLIMB = new Autonomus("Feeding Climb", new FeedingBack(), SwerveAutoFollower.getStartPose("FC1"), 180);
        public static final Autonomus FEEDING_SHOOTING = new Autonomus("Feeding Shooting", new FeedingShooting(), SwerveAutoFollower.getStartPose("FS1"), 180);
        public static final Autonomus MAGAZINE_CLIMB_RIGHT = new Autonomus("Magazine Climb Right", new MgazineClimbRight(), SwerveAutoFollower.getStartPose("MCR1"), 180);
        public static final Autonomus MAGAZINE_CLIMB = new Autonomus("Magazine Climb", new MagazineClimb(), SwerveAutoFollower.getStartPose("MC1"), 0);



        public static final MRobotState IDLE = new MRobotState("IDLE",
                        IntakeRollerConstants.IDLE, SixBarConstants.IDLE, ShooterConstants.IDLE, SandwichConstants.IDLE,
                        RollerConstants.IDLE, HoodConstants.IDLE, TransferConstants.IDLE,
                        KickerConstants.IDLE);

        public static final MRobotState IDLE_INTAKE = new MRobotState("IDLE_INTAKE",
                        IntakeRollerConstants.IDLE, SandwichConstants.IDLE,
                        RollerConstants.IDLE, HoodConstants.IDLE, TransferConstants.IDLE,
                        KickerConstants.IDLE);

        public static final MRobotState IDLE_SHOOTER = new MRobotState("IDLEֹֹ_SHOOTER",
                        ShooterConstants.IDLE, HoodConstants.IDLE);

        public static final MRobotState INTAKE_DEPLOY = new MRobotState("INTAKE_DEPLOY",
                        IntakeRollerConstants.FORWARD, SandwichConstants.INTAKE, SixBarConstants.DEPLOY,
                        RollerConstants.INTAKE, TransferConstants.INTAKE, KickerConstants.INTAKE);

        public static final MRobotState INTAKE_ROLLER = new MRobotState("INTAKE_ROLLER",
                        IntakeRollerConstants.FORWARD, SandwichConstants.INTAKE, RollerConstants.INTAKE,
                        SixBarConstants.ARMBRAKS,
                        TransferConstants.INTAKE, KickerConstants.INTAKE);

        public static final MRobotState FEEDING_IN_MOTION = new MRobotState("FEEDING_IN_MOTION", () -> {
                if (SixBar.getInstance().getCurrentState() != SixBarConstants.ARMBRAKS) {
                        SixBar.getInstance().setState(SixBarConstants.DEPLOY);
                }
                SuperStructure.atPointLatch = false;
        },

                        ShooterConstants.FEEDING_IN_MOTION, SandwichConstants.FEEDING_IN_MOTION,
                        RollerConstants.FEEDING_IN_MOTION, TransferConstants.FEEDING_IN_MOTION,
                        HoodConstants.FEEDING_IN_MOTION, IntakeRollerConstants.FORWARD,
                        KickerConstants.FEEDING_IN_MOTION);

        public static final MRobotState FEEDING = new MRobotState("FEEDING", () -> {
                SuperStructure.atPointLatch = false;
                SwerveController.atPointLock = false;
        }, () -> {
                SixBar.getInstance(     ).setState(SixBar.getInstance().getLastState());
                SwerveController.atPointLock = false;
        },
                        ShooterConstants.FEEDING, SandwichConstants.FEEDING,
                        RollerConstants.FEEDING, TransferConstants.FEEDING, HoodConstants.FEEDING,
                        SixBarConstants.SHOOTING, IntakeRollerConstants.IDLE, KickerConstants.FEEDING);

        public static final MRobotState SHOOTING = new MRobotState("SHOOTING", () -> {
                SwerveController.isAbs = 0;
                SuperStructure.atPointLatch = false;
                SuperStructure.isLocked = false;
                SuperStructure.ballsShot = 0;
               SuperStructure.startShootingTime = Timer.getFPGATimestamp();
               SwerveController.atPointLock = false;
               Robot.counter = 0;
               SandwichCommand.startTimer.stop();
               SandwichCommand.startTimer.reset();
               Sandwich.getInstance().resetIntakeLach();


        }, () -> {
                SixBar.getInstance().setState(SixBar.getInstance().getLastState());
                SwerveController.atPointLock = false;
                SwerveController.isAbs = 0;
        },
                        ShooterConstants.SHOOTING, SandwichConstants.SHOOTING,
                        RollerConstants.SHOOTING, TransferConstants.SHOOTING, HoodConstants.SHOOTING,
                        SixBarConstants.SHOOTING, IntakeRollerConstants.SHOOTING, KickerConstants.SHOOTING);

        public static final MRobotState SHOOTING_UNLOCKED = new MRobotState("SHOOTING_UNLOCKED", () -> {
                SwerveController.isAbs = 0;
                SuperStructure.atPointLatch = false;
                SuperStructure.isLocked = false;
                SuperStructure.ballsShot = 0;
               SuperStructure.startShootingTime = Timer.getFPGATimestamp();
               SwerveController.atPointLock = false;
               Robot.counter = 0;
               SandwichCommand.startTimer.stop();
               SandwichCommand.startTimer.reset();
               


        }, () -> {
                SixBar.getInstance().setState(SixBar.getInstance().getLastState());
                SwerveController.atPointLock = false;
                SwerveController.isAbs = 0;
        },
                        ShooterConstants.SHOOTING, SandwichConstants.SHOOTING,
                        RollerConstants.SHOOTING, TransferConstants.SHOOTING, HoodConstants.SHOOTING,
                        SixBarConstants.DEPLOY, IntakeRollerConstants.FORWARD, KickerConstants.SHOOTING);

        public static final MRobotState SHOOTING_PRESETS = new MRobotState("SHOOTING_PRESETS", () -> {
                SuperStructure.atPointLatch = false;
        }, () -> {
                SixBar.getInstance().setState(SixBar.getInstance().getLastState());
        },
                        ShooterConstants.SHOOTING, SandwichConstants.SHOOTING, RollerConstants.SHOOTING,
                        TransferConstants.SHOOTING,
                        HoodConstants.SHOOTING, SixBarConstants.SHOOTING, IntakeRollerConstants.SHOOTING,
                        KickerConstants.SHOOTING);

        public static final MRobotState EJECT = new MRobotState("EJECT", () -> {
        }, () -> SixBar.getInstance().setState(SixBar.getInstance().getLastState()),
                        SandwichConstants.EJECT, RollerConstants.EJECT, TransferConstants.EJECT, HoodConstants.EJECT,
                        ShooterConstants.EJECT, KickerConstants.EJECT, SixBarConstants.SHOOTING, IntakeRollerConstants.SHOOTING);

        public static final MRobotState INTAKE_EJECT = new MRobotState("INTAKE_EJECT", () -> {
        }, SixBarConstants.DEPLOY, IntakeRollerConstants.BACKWARD);

        public static final MRobotState UNSTUCK = new MRobotState("UNSTUCK",
                        SandwichConstants.UNSTUCK, RollerConstants.UNSTUCK, TransferConstants.UNSTUCK,
                        IntakeRollerConstants.IDLE, KickerConstants.UNSTUCK);

        public static final MRobotState PRECLIMB = new MRobotState("PRECLIMB",() -> {ClimbCommand.isAtPosition = false;
        },
                        Climb.PRECLIMB, SixBarConstants.DEPLOY, IntakeRollerConstants.IDLE, HoodConstants.IDLE,
                        ShooterConstants.IDLE, SandwichConstants.IDLE, RollerConstants.IDLE, TransferConstants.IDLE,
                        KickerConstants.IDLE);

        public static final MRobotState CLIMB = new MRobotState("CLIMB",
                        Climb.CLIMB, SixBarConstants.DEPLOY, IntakeRollerConstants.IDLE, HoodConstants.IDLE,
                        ShooterConstants.IDLE, SandwichConstants.IDLE, RollerConstants.IDLE, TransferConstants.IDLE,
                        KickerConstants.IDLE);
}
