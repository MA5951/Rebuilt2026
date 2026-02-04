
package frc.robot;

import com.MAutils.RobotControl.MRobotState;

import frc.robot.Commands.SwerveController;
import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Subsystems.Hood.HoodConstants;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.Roller.RollerConstants;
import frc.robot.Subsystems.Sandwich.SandwichConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.SixBar.SixBar;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Transfer.TransferConstants;

public class RobotConstants {

        public static final MRobotState IDLE = new MRobotState("IDLE",
                        IntakeRollerConstants.IDLE, SixBarConstants.IDLE, ShooterConstants.IDLE, SandwichConstants.IDLE,
                        RollerConstants.IDLE, HoodConstants.IDLE, ClimbConstnats.IDLE, TransferConstants.IDLE);

        public static final MRobotState IDLE_INTAKE = new MRobotState("IDLE_INTAKE",
                        IntakeRollerConstants.IDLE, SandwichConstants.IDLE,
                        RollerConstants.IDLE, HoodConstants.IDLE, TransferConstants.IDLE, ClimbConstnats.IDLE);

        public static final MRobotState IDLE_SHOOTER = new MRobotState("IDLEֹֹ_SHOOTER",
                        ShooterConstants.IDLE, HoodConstants.IDLE);

        public static final MRobotState INTAKE_DEPLOY = new MRobotState("IDLE_DEPLOY",
                        IntakeRollerConstants.FORWARD, SandwichConstants.INTAKE, SixBarConstants.DEPLOY,
                        RollerConstants.INTAKE, TransferConstants.INTAKE);

        public static final MRobotState INTAKE_ROLLER = new MRobotState("INTAKE_ROLLER",
                        IntakeRollerConstants.FORWARD, SandwichConstants.INTAKE, RollerConstants.INTAKE, SixBarConstants.ARMBRAKS,
                        TransferConstants.INTAKE);

        public static final MRobotState FEEDING_IN_MOTION = new MRobotState("FEEDING_IN_MOTION", () -> {
                if (SixBar.getInstance().getCurrentState() != SixBarConstants.ARMBRAKS) {
                        SixBar.getInstance().setState(SixBarConstants.DEPLOY);
                }
        },

                        ShooterConstants.FEEDING_IN_MOTION, SandwichConstants.FEEDING_IN_MOTION,
                        RollerConstants.FEEDING_IN_MOTION, TransferConstants.FEEDING_IN_MOTION,
                        HoodConstants.FEEDING_IN_MOTION,IntakeRollerConstants.FORWARD);

        public static final MRobotState FEEDING = new MRobotState("FEEDING",() -> {},() -> {
                SixBar.getInstance().setState(SixBar.getInstance().getLastState());},
                        ShooterConstants.FEEDING, SandwichConstants.FEEDING,
                        RollerConstants.FEEDING, TransferConstants.FEEDING, HoodConstants.FEEDING,
                        SixBarConstants.SHOOTING, IntakeRollerConstants.IDLE);

        public static final MRobotState SHOOTING = new MRobotState("SHOOTING",() -> {SwerveController.isAbs = false;},() -> {
                SixBar.getInstance().setState(SixBar.getInstance().getLastState());},
                        ShooterConstants.SHOOTING, SandwichConstants.SHOOTING,
                        RollerConstants.SHOOTING, TransferConstants.SHOOTING, HoodConstants.SHOOTING,
                        SixBarConstants.SHOOTING,IntakeRollerConstants.IDLE);

        public static final MRobotState SHOOTING_PRESETS = new MRobotState("SHOOTING_PRESETS",() -> {},() -> {
                SixBar.getInstance().setState(SixBar.getInstance().getLastState());},
                        ShooterConstants.SHOOTING, SandwichConstants.SHOOTING, RollerConstants.SHOOTING,
                        TransferConstants.SHOOTING,
                        HoodConstants.SHOOTING, SixBarConstants.SHOOTING, IntakeRollerConstants.IDLE);

        public static final MRobotState EJECT = new MRobotState("EJECT",() -> {},() -> {
                SixBar.getInstance().setState(SixBar.getInstance().getLastState());},
                        SandwichConstants.EJECT, RollerConstants.EJECT, TransferConstants.EJECT, HoodConstants.EJECT, ShooterConstants.EJECT);

        public static final MRobotState UNSTUCK = new MRobotState("UNSTUCK",
                        SandwichConstants.UNSTUCK, RollerConstants.UNSTUCK, TransferConstants.UNSTUCK,
                         IntakeRollerConstants.IDLE);

        public static final MRobotState PRECLIMB = new MRobotState("PRECLIMB",
                        ClimbConstnats.PRECLIMB, SixBarConstants.DEPLOY, IntakeRollerConstants.IDLE, HoodConstants.IDLE,
                        ShooterConstants.IDLE, SandwichConstants.IDLE, RollerConstants.IDLE, TransferConstants.IDLE);

        public static final MRobotState CLIMB = new MRobotState("CLIMB",
                        ClimbConstnats.CLIMB, SixBarConstants.DEPLOY, IntakeRollerConstants.IDLE, HoodConstants.IDLE,
                        ShooterConstants.IDLE, SandwichConstants.IDLE, RollerConstants.IDLE, TransferConstants.IDLE);

}
