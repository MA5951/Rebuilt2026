
package frc.robot;

import com.MAutils.CanBus.CANBusID;
import com.ctre.phoenix6.CANBus;

public class PortMap {

    public class CAN_BUS {
        public static final CANBus RIO_BUS = new CANBus("rio");
        public static final CANBus CANIVORE_BUS = new CANBus("*");

        
    }

    public static class HoodPorts {
        public static final CANBusID HOOD_MOTOR = new CANBusID(1, CAN_BUS.RIO_BUS);
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
        }

    


}
