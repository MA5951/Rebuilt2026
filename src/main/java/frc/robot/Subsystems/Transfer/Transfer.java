
package frc.robot.Subsystems.Transfer;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Shooter.Shooter;

public class Transfer extends PowerControlledSystem {
        private static Transfer transfer;

        private Transfer() {
                super(TransferConstants.TRANSFER_CONSTANTS, TransferConstants.IDLE, TransferConstants.INTAKE,
                                TransferConstants.FEEDING,
                                TransferConstants.FEDDING_IN_MOTION, TransferConstants.EJECT,
                                TransferConstants.SHOOTING, TransferConstants.UNSTUCK);
        }

        @Override
        public void createSelfTest() {

        }

        private boolean canShoot() {
                return (Swerve.atPointForSHooting() && hood.atPointForSHooting() && Shooter.atPointForSHooting()) &&
                        (RobotContainer.getRobotState() == RobotConstants.SHOOTING 
                        || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS
                        || RobotContainer.getRobotState() == RobotConstants.FEEDING);// TODO After Merge
        }

        private boolean canFeedingInMotion() {
                return (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION && hood.atPointForFeeding()
                                && Shooter.atPointForFeeding && !SuperStructure.isHittingNet()
                                && !SuperStructur.outSideField());// TODO After Merge
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
                                || RobotContainer.getRobotState() == RobotConstants.IDLE; // TODO After Merge

        }

        public static Transfer getInstance() {
                if (transfer == null) {
                        transfer = new Transfer();
                }
                return transfer;
        }

}
