
package frc.robot.Subsystems.Transfer;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;

public class Transfer extends PowerControlledSystem {
        private static Transfer transfer;

        private Transfer() {
                super(TransferConstants.TRANSFER_CONSTANTS, TransferConstants.IDLE, TransferConstants.INTAKE,
                                TransferConstants.FEEDING,
                                TransferConstants.FEEDING_IN_MOTION, TransferConstants.EJECT,
                                TransferConstants.SHOOTING, TransferConstants.UNSTUCK);
        }

        @Override
        public void createSelfTest() {

        }

        private boolean canShoot() {
                return (Swerve.getInstance().atPointForShooting() && Hood.getInstance().atPoint()
                                && Shooter.getInstance().atPoint()) && // TODO change to larger tolerance
                                (RobotContainer.getRobotState() == RobotConstants.SHOOTING
                                                || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS
                                                || RobotContainer.getRobotState() == RobotConstants.FEEDING);
        }

        private boolean canFeedingInMotion() {
                return (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION
                                && Hood.getInstance().atPoint()
                                && Shooter.getInstance().atPoint() && !SuperStructure.isHittingNet()
                                && !SuperStructure.outSideField());// TODO change to larger tolerance
        }

        @Override
        public boolean CAN_MOVE() {
                return canShoot() || canFeedingInMotion()
                                || RobotContainer.getRobotState() == RobotConstants.INTAKE_DEPLOY
                                || RobotContainer.getRobotState() == RobotConstants.INTAKE_ROLLER
                                || RobotContainer.getRobotState() == RobotConstants.UNSTUCK
                                || RobotContainer.getRobotState() == RobotConstants.EJECT
                                || RobotContainer.getRobotState() == RobotConstants.IDLE_INTAKE
                                || RobotContainer.getRobotState() == RobotConstants.IDLE_SHOOTER
                                || RobotContainer.getRobotState() == RobotConstants.IDLE;

        }

        public static Transfer getInstance() {
                if (transfer == null) {
                        transfer = new Transfer();
                }
                return transfer;
        }

}
