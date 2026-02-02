
package frc.robot;

import com.MAutils.CanBus.CANBusID;
import com.MAutils.CanBus.SwerveModuleID;
import com.ctre.phoenix6.CANBus;

public class PortMap {

    public class CAN_BUS {
        public static final CANBus RIO_BUS = new CANBus("rio");
        public static final CANBus CANIVORE_BUS = new CANBus("*");

        
    }

    public static class HoodPorts {
        public static final CANBusID HOOD_MOTOR = new CANBusID(1, CAN_BUS.RIO_BUS);
        public static final int CAN_CODER = 0;
    }

    public class SixBarPorts {
        public static final CANBusID SIXBAR_MOTOR = new CANBusID(1, CAN_BUS.RIO_BUS);
        public static final int CAN_CODER = 0;
    }
    
    public class Sandwich_Ports {
        public static final CANBusID SANDWICH_MOTOR = new CANBusID(15, CAN_BUS.RIO_BUS);
        public static final int MACAM = 11;
        public static final int IR = 1;
    }
    public class RollerPorts {
        public static final CANBusID ROLLER_MOTOR = new CANBusID(21, CAN_BUS.RIO_BUS);
    }
public class Intake_Roller_Ports{
            public static final CANBusID INTAKE_ROLLER_MOTOR = new CANBusID(0, CAN_BUS.RIO_BUS);
        }
    public class ClimbPorts{
            public static final CANBusID CLIMB_MOTOR = new CANBusID(0, CAN_BUS.RIO_BUS);
            public static final int MACAM = 0;
        }

public class Transfer_Ports {
        public static final CANBusID TRANSFER_MOTOR = new CANBusID(11, CAN_BUS.RIO_BUS);

        public static final int FIRST_LEVEL_SENSOR = 0;
        public static final int SECOND_LEVEL_SENSOR = 1;
        public static final int THIRD_LEVEL_SENSOR = 2;
}

    

    public static class SwervePorts {

        private static final CANBusID LEFT_FRONT_ENCODER = new CANBusID(22, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID LEFT_FRONT_DRIVE = new CANBusID(8, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID LEFT_FRONT_TURNING = new CANBusID(5, CAN_BUS.CANIVORE_BUS);

  
        private static final CANBusID LEFT_BACK_ENCODER = new CANBusID(21, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID LEFT_BACK_DRIVE = new CANBusID(4, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID LEFT_BACK_TURNING = new CANBusID(9, CAN_BUS.CANIVORE_BUS);

        private static final CANBusID RIGHT_FRONT_ENCODER = new CANBusID(23, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID RIGHT_FRONT_DRIVE = new CANBusID(7, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID RIGHT_FRONT_TURNING = new CANBusID(6, CAN_BUS.CANIVORE_BUS);

        private static final CANBusID RIGHT_BACK_ENCODER = new CANBusID(24, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID RIGHT_BACK_DRIVE = new CANBusID(2, CAN_BUS.CANIVORE_BUS);
        private static final CANBusID RIGHT_BACK_TURNING = new CANBusID(3, CAN_BUS.CANIVORE_BUS);
        public static final CANBusID PIGEON2 = new CANBusID(12, CAN_BUS.CANIVORE_BUS);

        public static final SwerveModuleID[] SWERVE_MODULE_IDS = {
                new SwerveModuleID(LEFT_FRONT_DRIVE, LEFT_FRONT_TURNING, LEFT_FRONT_ENCODER),
                new SwerveModuleID(RIGHT_FRONT_DRIVE, RIGHT_FRONT_TURNING, RIGHT_FRONT_ENCODER),
                new SwerveModuleID(LEFT_BACK_DRIVE, LEFT_BACK_TURNING, LEFT_BACK_ENCODER),

                new SwerveModuleID(RIGHT_BACK_DRIVE, RIGHT_BACK_TURNING, RIGHT_BACK_ENCODER)
        };

    }


    public class ShooterPorts {
        public static final CANBusID SHOOTER_MASTER = new CANBusID(1, CAN_BUS.RIO_BUS);
        public static final CANBusID SHOOTER_SLAVE = new CANBusID(2, CAN_BUS.RIO_BUS);
        public static final int IR = 0;
    }

}
