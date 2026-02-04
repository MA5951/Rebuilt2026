
package frc.robot.Subsystems.Transfer;

import com.MAutils.Components.MACam;
import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Systems.PowerControlledSystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap.Transfer_Ports;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;

public class Transfer extends PowerControlledSystem {
        private static Transfer transfer;

        private MACam firstLevelSensor;
        private MACam secondLevelSensor;
        private MACam thirdLevelSensor;

        private Transfer() {
                super(TransferConstants.TRANSFER_CONSTANTS, TransferConstants.IDLE, TransferConstants.INTAKE,
                                TransferConstants.FEEDING,
                                TransferConstants.FEEDING_IN_MOTION, TransferConstants.EJECT,
                                TransferConstants.SHOOTING, TransferConstants.UNSTUCK);

                firstLevelSensor = new MACam(Transfer_Ports.FIRST_LEVEL_SENSOR);
                secondLevelSensor = new MACam(Transfer_Ports.SECOND_LEVEL_SENSOR);
                thirdLevelSensor = new MACam(Transfer_Ports.THIRD_LEVEL_SENSOR);
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

        public double getFirstLevelSensorDistance() {
                return firstLevelSensor.getDistance();
        }

        public double getSecondLevelSensorDistance() {
                return secondLevelSensor.getDistance();
        }

        public double getThirdLevelSensorDistance() {
                return thirdLevelSensor.getDistance();
        }

        public void periodic() {

                MALog.log("Subsystems/Transfer/First Level Sensor Distance", getFirstLevelSensorDistance());
                MALog.log("Subsystems/Transfer/Second Level Sensor Distance", getSecondLevelSensorDistance());
                MALog.log("Subsystems/Transfer/Third Level Sensor Distance", getThirdLevelSensorDistance());
                super.periodic();
        }

        public static Transfer getInstance() {
                if (transfer == null) {
                        transfer = new Transfer();
                }
                return transfer;
        }

}
