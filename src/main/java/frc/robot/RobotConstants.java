
package frc.robot;

import com.MAutils.RobotControl.MRobotState;

import frc.robot.Subsystems.Climb.ClimbConstnats;
import frc.robot.Subsystems.Hood.HoodConstants;
import frc.robot.Subsystems.IntakeRoller.IntakeRollerConstants;
import frc.robot.Subsystems.Roller.RollerConstants;
import frc.robot.Subsystems.Sandwich.SandwichConstants;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.SixBar.SixBarConstants;
import frc.robot.Subsystems.Transfer.TransferConstants;

public class RobotConstants {

    public static final MRobotState IDLE = new MRobotState("IDLE",
            IntakeRollerConstants.IDLE, SixBarConstants.IDLE, ShooterConstants.IDLE, SandwichConstants.IDLE,
            RollerConstants.IDLE, HoodConstants.IDLE, ClimbConstnats.IDLE, TransferConstants.IDLE);

    public static final MRobotState IDLE_INTAKE = new MRobotState("IDLE_INTAKE",
            IntakeRollerConstants.IDLE, SandwichConstants.IDLE,
            RollerConstants.IDLE, HoodConstants.IDLE, TransferConstants.IDLE);

    public static final MRobotState IDLE_SHOOTER = new MRobotState("IDLEֹֹ_SHOOTER",
            ShooterConstants.IDLE, HoodConstants.IDLE);

    public static final MRobotState INTAKE_DEPLOY = new MRobotState("IDLE_DEPLOY",
            IntakeRollerConstants.FORWARD, SandwichConstants.INTAKE, SixBarConstants.DEPLOY,
            RollerConstants.INTAKE, TransferConstants.INTAKE);

    public static final MRobotState INTAKE_ROLLER = new MRobotState("IDLE_DEPLOY",
            IntakeRollerConstants.FORWARD, SandwichConstants.INTAKE, RollerConstants.INTAKE, TransferConstants.INTAKE);

    public static final MRobotState FEEDING_IN_MOTION = new MRobotState("FEEDING_IN_MOTION",
            ShooterConstants.FEEDING_IN_MOTION, SandwichConstants.FEEDING_IN_MOTION,
            RollerConstants.FEEDING_IN_MOTION, TransferConstants.FEEDING_IN_MOTION, HoodConstants.FEEDING_IN_MOTION,
            SixBarConstants.DEPLOY, IntakeRollerConstants.FORWARD);

    public static final MRobotState FEEDING = new MRobotState("FEEDING",
            ShooterConstants.FEEDING, SandwichConstants.FEEDING,
            RollerConstants.FEEDING, TransferConstants.FEEDING, HoodConstants.FEEDING, SixBarConstants.DEPLOY,
            IntakeRollerConstants.FORWARD);

    public static final MRobotState SHOOTING = new MRobotState("SHOOTING",
            ShooterConstants.SHOOTING, SandwichConstants.SHOOTING,
            RollerConstants.SHOOTING, TransferConstants.SHOOTING, HoodConstants.SHOOTING, SixBarConstants.IDLE,
            IntakeRollerConstants.IDLE);

    public static final MRobotState SHOOTING_PRESETS = new MRobotState("SHOOTING_PRESETS",
            ShooterConstants.SHOOTING, SandwichConstants.SHOOTING, RollerConstants.SHOOTING, TransferConstants.SHOOTING,
            HoodConstants.SHOOTING, SixBarConstants.IDLE, IntakeRollerConstants.IDLE);

    public static final MRobotState EJECT = new MRobotState("EJECT",
            SandwichConstants.EJECT, RollerConstants.EJECT, TransferConstants.EJECT, HoodConstants.EJECT,
            SixBarConstants.IDLE, IntakeRollerConstants.IDLE);

    public static final MRobotState UNSTUCK = new MRobotState("UNSTUCK",
            SandwichConstants.UNSTUCK, RollerConstants.UNSTUCK, TransferConstants.UNSTUCK, HoodConstants.IDLE,
            ShooterConstants.IDLE, SixBarConstants.IDLE, IntakeRollerConstants.IDLE);

    public static final MRobotState PRECLIMB = new MRobotState("PRECLIMB",
            ClimbConstnats.PRECLIMB, SixBarConstants.IDLE, IntakeRollerConstants.IDLE, HoodConstants.IDLE,
            ShooterConstants.IDLE, SandwichConstants.IDLE, RollerConstants.IDLE, TransferConstants.IDLE);

    public static final MRobotState CLIMB = new MRobotState("CLIMB",
            ClimbConstnats.CLIMB, SixBarConstants.IDLE, IntakeRollerConstants.IDLE, HoodConstants.IDLE,
            ShooterConstants.IDLE, SandwichConstants.IDLE, RollerConstants.IDLE, TransferConstants.IDLE);

}
