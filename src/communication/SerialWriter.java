package communication;

import datatypes.GPS_Pos_t;
import datatypes.Waypoint_t;
import datatypes.s16;
import datatypes.u8;

public class SerialWriter {

	
	public SerialWriter(){
		
	}
	
	public void sendCommand(String frameType){	
		 switch (frameType) {

		    case "sendWP":
		    	//We define make a waypoint with some default data for testing
		    	
	    		/** (NAVI) SEND WAYPOINT **/
	    		
	    		Waypoint_t newWP = new Waypoint_t("new WP");
	    		newWP.Position.Longitude.value = 36827906; //1E-7 deg
	    		newWP.Position.Latitude.value = 510481055; //1E-7 deg
	    		newWP.Position.Altitude.value = 5038; //mm
	    		newWP.Position.Status.value = 1; //NEWDATA = 0x01
	    		newWP.Heading.value = 0; // orientation, 0 no action, 1...360 fix heading, neg. = Index to POI in WP List 
	    		newWP.ToleranceRadius.value = 10; // in meters, if the MK is within that range around the target, then the next target is triggered
	    		newWP.HoldTime.value = 2; // in seconds, if the was once in the tolerance area around a WP, this time defines the delay before the next WP is triggered
	    		newWP.Event_Flag.value = 1;// future implementation
	    		newWP.Index.value = 1; // to indentify different waypoints, workaround for bad communications PC <-> NC
	    		newWP.Type.value = 0;// typeof Waypoint 
	    		newWP.WP_EventChannelValue.value=100;//
	    		newWP.AltitudeRate.value = 30;	// rate to change the setpoint
	    		newWP.Speed.value = 30;// rate to change the Position
	    		newWP.CamAngle.value = 0;// Camera servo angle
	    		
	    		//encoder.send_command(2,'w',newWP.getAsInt());
		    	

		    	break;
		   
	    }
	}
}
