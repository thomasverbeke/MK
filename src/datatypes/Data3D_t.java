package datatypes;

import java.util.LinkedList;

/**
 * 3D Data Struct
 * adapted from: http://svn.mikrokopter.de/
 * navictrl - v0.28i - uart1.h
 */
public class Data3D_t extends c_int {
	 	public s16 AngleNick = new s16("AngleNick",-1800,1800);	// in 0.1 deg
	 	public s16 AngleRoll = new s16("AngleRoll",-1800,1800); // in 0.1 deg
	    public s16 Heading = new s16("Heading",-1800,1800); // in 0.1 deg
	    
	    public s8 StickNick = new s8("StickNick");
	    public s8 StickRoll = new s8("StickRoll");
	    public s8 StickYaw = new s8("StickYaw");
	    public u8 StickGas = new u8("StickGas");
	    
	    public u8 reserve[] = new u8[4];

	    public Data3D_t() {
	        allAttribs = new LinkedList<c_int>();
	        allAttribs.add(AngleNick);
	        allAttribs.add(AngleRoll);
	        allAttribs.add(Heading);
	        allAttribs.add(StickNick);
	        allAttribs.add(StickRoll);
	        allAttribs.add(StickRoll);
	        allAttribs.add(StickGas);
	        
	        for (int i = 0; i < reserve.length; i++) {
	            reserve[i] = new u8("reserve_" + i);
	            allAttribs.add(reserve[i]);
	        }
	    }
}
