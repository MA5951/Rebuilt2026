
package frc.robot;

import com.MAutils.CanBus.CANBusID;
import com.ctre.phoenix6.CANBus;

public class PortMap {

    public class CAN_BUS {
        public static final CANBus RIO_BUS = new CANBus("rio");
        public static final CANBus CANIVORE_BUS = new CANBus("*");
    }

    public class Transfer_Ports {
        public static final CANBusID TRANSFER_MOTOR = new CANBusID(11, CAN_BUS.RIO_BUS);
    }

}
