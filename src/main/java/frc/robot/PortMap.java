
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

    public class ClimbPorts{
            public static final CANBusID CLIMB_MOTOR = new CANBusID(0, CAN_BUS.RIO_BUS);
        }

    


}
