package datatypes;

import java.util.Arrays;
import java.util.LinkedList;


/**
 * Waypoint Data Struct
 * adapted from: http://svn.mikrokopter.de/
 * navictrl - v0.28i - waypoints.h
 */
public class Waypoint_t extends c_int {
	
    public final int POINT_TYPE_INVALID = 255;
    public final int POINT_TYPE_WP = 0;
    public final int POINT_TYPE_POI = 1;
    
    public GPS_Pos_t Position;             	// the gps position of the waypoint, see ubx.h for details
    public s16 Heading;                    	// orientation, 0 no action, 1...360 fix heading, neg. = Index to POI in WP List 
    public u8 ToleranceRadius;    			// in meters, if the MK is within that range around the target, then the next target is triggered
    public u8 HoldTime;                   	// in seconds, if the was once in the tolerance area around a WP, this time defines the delay before the next WP is triggered
    public u8 Event_Flag;                 	// future implementation
    public u8 Index;              		  	// to indentify different waypoints, workaround for bad communications PC <-> NC
    public u8 Type;                        	// typeof Waypoint
    public u8 WP_EventChannelValue;  		//
    public u8 AltitudeRate;					// rate to change the setpoint
    public u8 Speed;						// rate to change the Position
    public u8 CamAngle;						// Camera servo angle
    public u8 Name[];  						// Name of that point (ASCII) (4 u8)
    public u8 reserve[];             		// reserve (2 u8)
   
    
    //Check GPS_Pos_t => contains status flag which can be invalid, newdata or processed
    public static final int INVALID = 0x00;
    public static final int NEWDATA = 0x01;
    public static final int PROCESSED = 0x02;
    
    public Waypoint_t(String wp_name) {
        super(); //invoke superclass constructor
        name = wp_name;
        allAttribs = new LinkedList<c_int>();
        Position = new GPS_Pos_t(wp_name + " Position");
        Heading = new s16(wp_name + " Heading");
        ToleranceRadius = new u8(wp_name + " ToleranceRadius");
        HoldTime = new u8(wp_name + " HoldTime");
        Event_Flag = new u8(wp_name + " Event_Flag");
        Index = new u8(wp_name + " Index");
        Type = new u8(wp_name + " Type");
        WP_EventChannelValue = new u8(wp_name + " WP_EventChannelValue");
        AltitudeRate = new u8(wp_name + " AltitudeRate");
        Speed = new u8(wp_name + " Speed");
        CamAngle = new u8(wp_name + " CameraAngle");
        
        Name = new u8[4]; 
        for (int i = 0; i < Name.length; i++) {
        	Name[i] = new u8("");
        }
        reserve = new u8[2];
        for (int i = 0; i < reserve.length; i++) {
            reserve[i] = new u8(wp_name + "" + i);
        }
        allAttribs.add(Position);
        allAttribs.add(Heading);
        allAttribs.add(ToleranceRadius);
        allAttribs.add(HoldTime);
        allAttribs.add(Event_Flag);
        allAttribs.add(Index);
        allAttribs.add(Type);
        allAttribs.add(WP_EventChannelValue);
        allAttribs.add(AltitudeRate);
        allAttribs.add(Speed);
        allAttribs.add(CamAngle);
        allAttribs.addAll(Arrays.asList(Name)); 		//Returns a fixed-size list backed by the specified array.
        allAttribs.addAll(Arrays.asList(reserve));		//Returns a fixed-size list backed by the specified array.
    }
}
