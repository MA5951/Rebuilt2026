
package frc.robot;

import com.MAutils.CanBus.CANBusID;
import com.ctre.phoenix6.CANBus;

public class PortMap {

    public class CAN_BUS {
        public static final CANBus RIO_BUS = new CANBus("rio");
        public static final CANBus CANIVORE_BUS = new CANBus("*");
    }

    public class SixBarPorts {
        public static final CANBusID SIXBAR_MOTOR = new CANBusID(1, CAN_BUS.RIO_BUS);
    }
    

}
