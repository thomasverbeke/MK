package datatypes;

import java.util.LinkedList;

/**
 * GPS-Data struct
 * adapted from: http://svn.mikrokopter.de/
 * navictrl - v0.28i - ubx.h
 */
public class GPS_Pos_t extends c_int {
	
		public final byte INVALID = 0x00;
		public final byte NEWDATA = 0x01;
		public final byte PROCESSED = 0x02;
	
		public s32 Longitude;		// in 1E-7 deg
		public s32 Latitude;		// in 1E-7 deg
		public s32 Altitude; 		// in mm
		public u8 Status; 			// validity of data
		
		// The data are valid if the GPSData.Status is NEWDATA or PROCESSED.
		// To achieve new data after reading the GPSData.Status should be set to PROCESSED.

	    public GPS_Pos_t(String wp_name) {
	        super();
	        name = wp_name;
	        Longitude = new s32(wp_name + " Longitude");// in 1E-7 deg
	        Latitude = new s32(wp_name + " Latitude");// in 1E-7 deg
	        Altitude = new s32(wp_name + " Altitude"); // in mm
	        Altitude.minValue = 300; //not below 0.3 m?
	        Altitude.maxValue = 50000; //not above 50m?        
	        Status = new u8(wp_name + " Status"); // validity of data
	        allAttribs = new LinkedList<c_int>();
	        allAttribs.add(Longitude);
	        allAttribs.add(Latitude);
	        allAttribs.add(Altitude);
	        allAttribs.add(Status);
	    }

	    //TODO Why override?
	    
	    @Override
	    public int[] getAsInt() {
	        int[] ret = c_int.concatArray(Longitude.getAsInt(), Latitude.getAsInt());
	        ret = c_int.concatArray(ret, Altitude.getAsInt());
	        ret = c_int.concatArray(ret, Status.getAsInt());
	        return ret;
	    }
}
