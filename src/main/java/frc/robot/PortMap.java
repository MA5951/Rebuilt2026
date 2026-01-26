
package frc.robot;

import com.MAutils.CanBus.CANBusID;
import com.ctre.phoenix6.CANBus;

public class PortMap {

    public class CAN_BUS {
        public static final CANBus RIO_BUS = new CANBus("rio");
        public static final CANBus CANIVORE_BUS = new CANBus("*");
    }


    public class ShooterPorts {
        public static final CANBusID SHOOTER_MASTER = new CANBusID(1, CAN_BUS.RIO_BUS);
        public static final CANBusID SHOOTER_SLAVE = new CANBusID(2, CAN_BUS.RIO_BUS);
    }

}
