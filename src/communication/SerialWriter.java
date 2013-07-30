package communication;

import datatypes.GPS_Pos_t;
import datatypes.Waypoint_t;
import datatypes.s16;
import datatypes.u16;
import datatypes.u8;

public class SerialWriter {

	/** The main purpose of this class it to send serial command to the MK.
	 *  We abstract the communication layer from the encoding/CRC check aspects to a simple
	 *  sendCommand function. For this simple command line application data send will be filled in by the application
	 *  to keep things simple.**/
	
	public SerialWriter(){
		
	}
	
	public void sendCommand(String frameType){	
		 switch (frameType) {

		 	//acces motordata => 'k'
		 	//version info => 'v'
		 	//debug data => 'd'
		 	//set/get NC-Parameter => 'j'
		 	//redirect debug uart => 'u'
		 	//new target positions => 's'
		 	//request for the text of the error status => 'e'
		 	//ftp command => 'f' => new feature: as of yet undocumented but found inside the firmware. 
		 	
		 	case "errorText":
		 		/** (NAVI) ERROR TEXT REQUSET **/
		 		//encoder.send_command(2,'e',null);
		 		
		 		break;
			 
		 	case "sendTarget":
		 		/** (NAVI) SEND TARGET POSITION**/
		 		
		 		/** question: what is difference between sending a target position and sending a waypoint?
		 		 * 	answer: from looking at the firmware: not a lot of difference; beeptime is shorter (50vs500ms) and there is no check for MaxNumberOfWaypoints
		 		 *  so the list is limited by the size of the array which is 101 big (100 WP max) (That's WP & POI combined).
		 		 * **/
	
		 		Waypoint_t wp_target = new Waypoint_t("new target");
		 		wp_target.Position.Longitude.value = 36827906; //1E-7 deg
		 		wp_target.Position.Latitude.value = 510481055; //1E-7 deg
		 		wp_target.Position.Altitude.value = 5038; //mm
		 		wp_target.Position.Status.value = wp_target.Position.NEWDATA;
		 		wp_target.Heading.value = 0; // orientation, 0 no action, 1...360 fix heading, neg. = Index to POI in WP List 
		 		wp_target.ToleranceRadius.value = 10; // in meters, if the MK is within that range around the target, then the next target is triggered
		 		wp_target.HoldTime.value = 2; // in seconds, if the was once in the tolerance area around a WP, this time defines the delay before the next WP is triggered
		 		wp_target.Event_Flag.value = 1;// future implementation
		 		wp_target.Index.value = 1; // to indentify different waypoints, workaround for bad communications PC <-> NC; start from 1!
		 		wp_target.Type.value = wp_target.POINT_TYPE_WP;// typeof Waypoint 
		 		wp_target.WP_EventChannelValue.value=100;//
		 		wp_target.AltitudeRate.value = 30;	// rate to change the setpoint
		 		wp_target.Speed.value = 30;// rate to change the Position
		 		wp_target.CamAngle.value = 0;// Camera servo angle
	    		
	    		//encoder.send_command(2,'s',wp_target.getAsInt());
		 		break;
		 
		    case "sendWP":
		    	//We define make a waypoint with some default data for testing
		    	
	    		/** (NAVI) SEND WAYPOINT **/
	    		
	    		Waypoint_t wp = new Waypoint_t("new WP");
	    		wp.Position.Longitude.value = 36827906; //1E-7 deg
	    		wp.Position.Latitude.value = 510481055; //1E-7 deg
	    		wp.Position.Altitude.value = 5038; //mm
	    		wp.Position.Status.value = wp.Position.NEWDATA;
	    		wp.Heading.value = 0; // orientation, 0 no action, 1...360 fix heading, neg. = Index to POI in WP List 
	    		wp.ToleranceRadius.value = 10; // in meters, if the MK is within that range around the target, then the next target is triggered
	    		wp.HoldTime.value = 2; // in seconds, if the was once in the tolerance area around a WP, this time defines the delay before the next WP is triggered
	    		wp.Event_Flag.value = 1;// future implementation
	    		wp.Index.value = 1; // to indentify different waypoints, workaround for bad communications PC <-> NC; start from 1!
	    		wp.Type.value = wp.POINT_TYPE_WP;// typeof Waypoint 
	    		wp.WP_EventChannelValue.value=100;//
	    		wp.AltitudeRate.value = 30;	// rate to change the setpoint
	    		wp.Speed.value = 30;// rate to change the Position
	    		wp.CamAngle.value = 0;// Camera servo angle
	    		
	    		//encoder.send_command(2,'w',wp.getAsInt());
		    	

		    	break;
		    	
		    case "sendWPlist":
		    	
		    	
		    	break;
		    	
		    case "reqWP":
		    	System.out.println("<SW>reqWP");
		    	
	    		/** (NAVI)REQUEST WP **/
	    		u8 WP = new u8("New WP");
	    		//Get WP 0
	    		WP.value = Long.valueOf(0).longValue();				    		
	    		//encoder.send_command(2,'x',WP.getAsInt());
		    	
		    	break;
		    	
		    case "serialTest":
		    	System.out.println("<SW>serialTest");
	    		/** (NAVI) SERIAL LINK TEST **/
	    		
	    		u16 echo = new u16("echo");
	    		//366 is our testvalue
	    		echo.value=Long.valueOf("366").longValue();
	    		//encoder.send_command(2,'z',echo.getAsInt());		    	
		    	break;
		    	
		    case "3DDataInterval":
		    	System.out.println("<SW>3DDataInterval (in steps of 10 ms)");
		    	
	    		/** (NAVI) SET 3D DATA INTERVAL**/
	    		u8 interval_3D = new u8("interval");
	    		
	    		//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
	    		interval_3D.value = Long.valueOf(50).longValue(); //PASSING 0 STOPS THE STREAM
	    		//encoder.send_command(2,'c',interval_3D.getAsInt());
		
		    	break;
		    	
		    case "OSDDataInterval":
		    	System.out.println("<SW>OSDDataInterval");
		    	
	    		/** (NAVI) SET OSD DATA INTERVAL**/
	    		u8 interval_OSD = new u8("interval");
	    		//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
	    		interval_OSD.value = Long.valueOf(50).longValue();
	    		//encoder.send_command(2,'o',interval_OSD.getAsInt());

		    	break;
		    	
		    case "EngineTest":
		    	System.out.println("<SW>EngineTest");

	    		//TODO Switch to flightctrl
	    		/** (FLIGHT) Engine Test **/
	    		int val = 0;
	    		String data = "10";
	    		
	    		if (Integer.parseInt(data) > 10 || Integer.parseInt(data) < 0 ){
	    			val = 10;
	    		} else {
	    			val = Integer.parseInt(data);
	    		}
	    		
	    		int motor[] = new int[16];
	    		motor[0] = val;
	    		motor[1] = val;
	    		motor[2] = val;
	    		motor[3] = val;
	    		motor[4] = val;
	    		motor[5] = val;
	    		//encoder.send_command(1,'t',motor);

		    	break;
		    	
		    case "DebugReqInterval":
		    	System.out.println("<SW>DebugReqInterval");
	    
    			/** Labels of Analog values request (for all 32 channels) **/
    			//encoder.send_command(0,'a',0);
	    	
		    	
		    	break;
		    	
	    }
	}
}
