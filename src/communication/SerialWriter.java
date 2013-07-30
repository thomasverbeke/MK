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
	
	/** NOTE: Documentation talks about 4s subscription while firmware 8s?
	 * 	Also atm external control/ftp has not been implemented
	 * TODO check http://forum.mikrokopter.de/topic-43135.html
	 * **/
	
	public void sendCommand(String frameType){	
		 switch (frameType) {
		 	
		 	case "version":
		 		//break is missing in the firmware; so it doesn't matter what address this frame is given (we take 2 for NaviCtrl)
		 		//encoder.send_command(2,'v',null);
			 break;
			 
		    	
		    case "DebugReqNameInterval":
		    	/** (NAVI) SET LABELS DEBUG INTERVAL**/
		    	System.out.println("<SERIALWRITER>DebugReqInterval");
	    
		    	/** Labels of Analog values request (for all 32 channels) **/
		    	//encoder.send_command(2,'a',0);
		    	
		    	//TODO Change this; SYSTEM FOR THIS? We need to request one after the other....
		    	break;
		    	
		    case "DebugReqDataInterval":
		    	u8 interval_DebugData = new u8("interval");
		    	//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
		    	interval_DebugData.value = Long.valueOf(50).longValue(); //PASSING 0 STOPS THE STREAM
		    	//encoder.send_command(2,'d',interval_DebugData.getAsInt());
		    	break;
		 
		 	case "ftpCmd":
		 		/** (NAVI) FTP**/
		    	System.out.println("<SERIALWRITER>FTP (not yet implemented)");
		    	
		 		/** NOTE: This functionality is not yet documented inside the serial command list; I noted it in the firmware however & it has been mentioned
		 		 * on the firmware update page also.
		 		 * **/
		 		
		 		//TODO Implement FTP feature
			 break;
			 
		 	case "externControll":
		 		/** (NAVI) EXTERNAL CONTROLL**/
		    	System.out.println("<SERIALWRITER>External Ctrl (not yet implemented)");
		    	
		 		/** Take direct ctrl of gas, angle nick, angle roll, yaw gyro. (See external ctrl struct) 
		 		 *  http://www.mikrokopter.de/ucwiki/en/MoteCtrl
		 		 *  MoteCtrl has an example of the use of the extern ctrl struct (which is not documented)
		 		 *  The following was taken from wiimote.h
		 		 *  struct str_ExternControl {
		 		 *  	unsigned char Digital[2];
		 		 *  	unsigned char RemoteButtons; 
		 		 *  	signed char   Nick;  
		 		 *  	signed char   Roll; 
		 		 *  	signed char   Gier;
		 		 *  	unsigned char Gas; 
		 		 *  	signed char   Hight;  
		 		 *  	unsigned char free; 
		 		 *  	unsigned char Frame;
		 		 *  	unsigned char Config;
		 		 *  }
		 		 *  
		 		 * **/
		 		break;
			 
		 	case "BLStatus":
		 		/** (NAVI) BL CTRL STATUS 
		 		 * 	BL Status contains Current, Temperature, MaxPWM, Status Data about the motors
		 		 * **/
		 		System.out.println("<SERIALWRITER>BL CTRL Status Interval (in steps of 10ms)");
	    		u8 interval_BL = new u8("interval");
	    		
	    		//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
	    		interval_BL.value = Long.valueOf(50).longValue(); //PASSING 0 STOPS THE STREAM
	    		//encoder.send_command(2,'k',interval_3D.getAsInt());
		 		
		 		break;
		 	
		 	case "setParam" : 
		 		/** (NAVI) SET/GET NC PARAMETER **/
		 		System.out.println("<SERIALWRITER>Set Param");
		 		//this is DISABLED; changing parameters while in operation could cause MK to CRASH!
		 		//if you want to implement this: create a special type
		 		//u8 get(0)/set(1)
		 		//u8 parameterID
		 		//s16 value
		 		//parse it to an int[]
		 		//encoder.send_command(2,'j',???);
		 		break;
		 
		 	case "redirectUART" :
		 		/** (NAVI) REDIRECT UART **/
		 		System.out.println("<SERIALWRITER>Redirect Uart");
			 	//param: 1 byte 0x00 for FC; OxO1 for MK3MAG; 0x02 for MKGPS according to http://www.mikrokopter.de/ucwiki/en/SerialProtocol
			 	//int[] param = new int[0];
			 	//param[0] = 0x00;
			 	//encoder.send_command(2,'u',param);
			 	//use magic packet to return to navi
			 	break;
		 
		 	case "errorText":
		 		/** (NAVI) ERROR TEXT REQUEST 
		 		 * 
		 		 * Respons is a char[] containing the error text**/
		 		System.out.println("<SERIALWRITER>errorText");
		 		//encoder.send_command(2,'e',null);
		 		
		 		break;
			 
		 	case "sendTarget":
		 		/** (NAVI) SEND TARGET POSITION**/
		 		
		 		/** question: what is difference between sending a target position and sending a waypoint?
		 		 * 	answer: from looking at the firmware: not a lot of difference; beeptime is shorter (50vs500ms) and there is no check for MaxNumberOfWaypoints
		 		 *  so the list is limited by the size of the array which is 101 big (100 WP max) (That's WP & POI combined).
		 		 * **/
		 		
		 		System.out.println("<SERIALWRITER>sendTarget");
	
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
		 		wp_target.WP_EventChannelValue.value=100;// Will be transferred to the FC and can be used as Poti value there
		 		wp_target.AltitudeRate.value = 30;	// rate to change the setpoint
		 		wp_target.Speed.value = 30;// rate to change the Position
		 		wp_target.CamAngle.value = 0;// Camera servo angle
	    		
	    		//encoder.send_command(2,'s',wp_target.getAsInt());
		 		break;
		 
		    case "sendWP":
		    	//We define make a waypoint with some default data for testing
		    	
	    		/** (NAVI) SEND WAYPOINT **/
		    	
		    	System.out.println("<SERIALWRITER>sendWP");
	    		
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
	    		wp.WP_EventChannelValue.value=100;// Will be transferred to the FC and can be used as Poti value there
	    		wp.AltitudeRate.value = 30;	// rate to change the setpoint
	    		wp.Speed.value = 30;// rate to change the Position
	    		wp.CamAngle.value = 0;// Camera servo angle
	    		
	    		//encoder.send_command(2,'w',wp.getAsInt());
		    	break;
		    	
		    case "sendWPlist":
		    	/** (NAVI) SEND WAYPOINT LIST **/
		    	//TODO 
		    	System.out.println("<SERIALWRITER>sendWPlist");
		    	
		    	break;
		    	
		    case "reqWP":
	    		/** (NAVI)REQUEST WP **/
		    	System.out.println("<SERIALWRITER>reqWP");
		    	
	    		u8 WP = new u8("New WP");
	    		//Get WP 0
	    		WP.value = Long.valueOf(0).longValue();				    		
	    		//encoder.send_command(2,'x',WP.getAsInt());
		    	
		    	break;
		    	
		    case "serialTest":
	    		/** (NAVI) SERIAL LINK TEST **/
		    	System.out.println("<SERIALWRITER>serialTest");
		    	
	    		u16 echo = new u16("echo");
	    		//366 is our testvalue
	    		echo.value=Long.valueOf("366").longValue();
	    		//encoder.send_command(2,'z',echo.getAsInt());		    	
		    	break;
		    	
		    case "3DDataInterval":
	    		/** (NAVI) SET 3D DATA INTERVAL**/
		    	System.out.println("<SERIALWRITER>3DDataInterval (in steps of 10 ms)");
		    	
	    		u8 interval_3D = new u8("interval");
	    		
	    		//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
	    		interval_3D.value = Long.valueOf(50).longValue(); //PASSING 0 STOPS THE STREAM
	    		//encoder.send_command(2,'c',interval_3D.getAsInt());
		
		    	break;
		    	
		    case "OSDDataInterval":
	    		/** (NAVI) SET OSD DATA INTERVAL**/
		    	
		    	System.out.println("<SERIALWRITER>OSDDataInterval (in steps of 10ms)");
	    		u8 interval_OSD = new u8("interval");
	    		//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
	    		interval_OSD.value = Long.valueOf(50).longValue();
	    		//encoder.send_command(2,'o',interval_OSD.getAsInt());

		    	break;
		    	
		    case "EngineTest":
		    	/** (FLIGHT) ENGINE TEST
		    	 * **/
		    	System.out.println("<SERIALWRITER>Redirect Uart");	    	
		    	
			 	//param: 1 byte 0x00 for FC; OxO1 for MK3MAG; 0x02 for MKGPS according to http://www.mikrokopter.de/ucwiki/en/SerialProtocol
			 	//int[] param = new int[0];
			 	//param[0] = 0x00;
			 	//encoder.send_command(2,'u',param);
			 	//use magic packet to return to navi
		    	
		    	System.out.println("<SERIALWRITER>EngineTest");

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

	    		//TODO send magic pakket!!
	    		
		    	break;

		    	
	    }
	}
}
