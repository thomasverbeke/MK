package datatypes;

import java.util.LinkedList;

/**
 * BLDataStruct
 * adapted from: http://svn.mikrokopter.de/
 * navictrl - 0.28i - spi_slave.h - here it is called Motor_t
 * flightctrl - 0.88n - twimaster.h - here it is called MotorData_t
 * 
 * These values come over SPI with FlightCtrl (comments have been taken from flightctrl)
 */
public class BLData_t extends c_int {
	public u8 Index;		//self not in spi_slave
	public u8 Current;		// in 0.1 A steps, read back from BL
	public u8 Temperature;	// old BL-Ctrl will return a 255 here, the new version the temp. in C
	public u8 MaxPWM;		// read back from BL -> is less than 255 if BL is in current limit, not running (250) or starting (40)
    public u8 State;		// 7 bit for I2C error counter, highest bit indicates if motor is present
	
    public BLData_t() {

        Index = new u8("Index");
        Current = new u8("Current");
        Temperature = new u8("Temperature");
        MaxPWM = new u8("MaxPWM");
        State = new u8("State");

        allAttribs = new LinkedList<c_int>();
        allAttribs.add(Index);
        allAttribs.add(Current);
        allAttribs.add(Temperature);
        allAttribs.add(MaxPWM);
        allAttribs.add(State);
  
    }
}