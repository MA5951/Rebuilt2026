
package frc.robot.Subsystems.Transfer;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;

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
                return (SuperStructure.atPointForShooting()) &&
                                (RobotContainer.getRobotState() == RobotConstants.SHOOTING
                                                || RobotContainer.getRobotState() == RobotConstants.SHOOTING_PRESETS);
        }

        private boolean canFeedingInMotion() {
                return (SuperStructure.atPointForFeedingInMotion()
                                && RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION);
        }

        private boolean canFeeding() {
                return (SuperStructure.atPointForFeeding() && RobotContainer.getRobotState() == RobotConstants.FEEDING);
        }

        @Override
        public boolean CAN_MOVE() {
                return canShoot() || canFeedingInMotion() || canFeeding()
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
