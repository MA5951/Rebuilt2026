
package frc.robot.Subsystems.Transfer;

import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap.Transfer_Ports;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;

public class Transfer extends PowerControlledSystem {
        private static Transfer transfer;

        private DigitalInput firstLevelSensor;
        private DigitalInput secondLevelSensor;
        private DigitalInput thirdLevelSensor;


        private Transfer() {
                super(TransferConstants.TRANSFER_CONSTANTS, TransferConstants.IDLE, TransferConstants.INTAKE,
                                TransferConstants.FEEDING,
                                TransferConstants.FEEDING_IN_MOTION, TransferConstants.EJECT,
                                TransferConstants.SHOOTING, TransferConstants.UNSTUCK);

                firstLevelSensor = new DigitalInput(Transfer_Ports.FIRST_LEVEL_SENSOR);
                secondLevelSensor = new DigitalInput(Transfer_Ports.SECOND_LEVEL_SENSOR);
                thirdLevelSensor = new DigitalInput(Transfer_Ports.THIRD_LEVEL_SENSOR);
        }

        @Override
        public void createSelfTest() {

        }

        public boolean isMoving() {
                return getVelocity() > TransferConstants.MAX_VELOCITY_IN_STOPING;
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

        public boolean isFirstLevelSensor() {
                return firstLevelSensor.get();
        }

        public boolean isSecondLevelSensor() {
                return secondLevelSensor.get();
        }

        public boolean isThirdLevelSensor() {
                return thirdLevelSensor.get();
        }

        public static Transfer getInstance() {
                if (transfer == null) {
                        transfer = new Transfer();
                }
                return transfer;
        }

}
