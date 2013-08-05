package communication;

import java.io.OutputStream;
import java.util.Timer;
import java.util.TimerTask;

import datatypes.Engine_t;
import datatypes.GPS_Pos_t;
import datatypes.Waypoint_t;
import datatypes.s16;
import datatypes.u16;
import datatypes.u8;

public class SerialWriter {

	/** The main purpose of this class it to send serial command to the MK.
	 *  We abstract the communication layer from the encoding/CRC check aspects to a simple
	 *  sendCommand function. For this simple command line application data send will be filled in by the application
	 *  to keep things simple
	 *  
	 *  Documentation talks about 4s subscription while firmware 8s?
	 *  
	 *  .**/
	public final int POINT_TYPE_INVALID = 255;
    public final int POINT_TYPE_WP = 0;
    public final int POINT_TYPE_POI = 1;
    
    /** Subscription Service
     * 
     * 	0 => DebugReqDataInterval
     *  1 => BLStatusInterval
     *  2 => SystemTimeInterval
     *  3 => 3DDataInterval
     *  4 => OSDInterval
    **/
	public int subscription[] = new int[5];
	Encoder encoder;
	
	public SerialWriter(OutputStream writer){
		if(writer!=null){
			encoder = new Encoder(writer);
		} else {
			System.out.println("Error: no outputstream defined in serialwriter constructor");
		}
	}
	
	
	public void _version(){
		//break is missing in the firmware; so it doesn't matter what address this frame is given (we take 2 for NaviCtrl)
		encoder.send_command(2,'v',0);
	}
	
	/** (NAVI) SET LABELS DEBUG INTERVAL
	 * @param indexAnalogLabel: index of labels of Analog values request (32 channels)
	 * **/
	public void _DebugReqName(int indexAnalogLabel){
    	System.out.println("<SERIALWRITER>DebugReq");
    	u8 index = new u8("indexAnalogLabel");
    	index.value = indexAnalogLabel; //TODO Test long->int conversions; because value is expecting a long; is right conversion happening?
    	encoder.send_command(2,'a',index.getAsInt());
	}

	/** 
	 * @param frameInterval, passing 0 stops the stream
	 * **/
	public void _DebugReqDataInterval(int frameInterval){
		System.out.println("<SERIALWRITER>Debug Data Interval (in steps of 10ms)");
		u8 intervalDebugData = new u8("frameInterval");
		intervalDebugData.value = frameInterval; 
    	encoder.send_command(2,'d',intervalDebugData.getAsInt());
    	subscription[0] = frameInterval;
	}
	
	/** (NAVI) BL CTRL STATUS 
	 * BL Status contains Current, Temperature, MaxPWM, Status Data about the motors
	 * @param frameInterval, passing 0 stops the stream
	 * **/
	public void _BLStatusInterval(int frameInterval){
 		System.out.println("<SERIALWRITER>BL CTRL Status Interval (in steps of 10ms)");
		u8 interval_BL = new u8("frameInterval");
		interval_BL.value = frameInterval;
		encoder.send_command(2,'k',interval_BL.getAsInt());
		subscription[1] = frameInterval;
	}
	
	/** (NAVI) REDIRECT UART 
	 * 	@param RedirectDestination 
	 * 	- 0x00 for FC
	 *  - OxO1 for MK3MAG
	 *  - 0x02 for MKGPS
	 *  Use "magic packet" to return to NAVI: 0x1B,0x1B,0x55,0xAA,0x00
	 * **/
	private void _redirectUART(byte RedirectDestination){	
 		System.out.println("<SERIALWRITER>Redirect Uart");
	 	int[] param = new int[0];
	 	param[0] = RedirectDestination;
	 	encoder.send_command(2,'u',RedirectDestination);
	}
	
	/** (NAVI) ERROR TEXT REQUEST 
	 * Response is a char[] containing the error text 
	 **/
	public void _errorText(){
 		System.out.println("<SERIALWRITER>errorText");
 		encoder.send_command(2,'e',0);
	}
	
	
	/** (NAVI) SEND TARGET POSITION 
	 *  question: What is difference between sending a target position and sending a waypoint?
	 * 	(short) answer: From looking at the firmware: not a lot of difference; beep time is shorter (50vs500ms) and there is no check for MaxNumberOfWaypoints
	 *  so the list is limited by the size of the array which is 101 big (100 WP max) (That's WP & POI combined).
	 *  
	 *  @param Longitude 			in 1E-7 deg
	 *  @param Latitude 			in 1E-7 deg
	 *  @param Altitude				in mm
	 *  @param Heading 				orientation, 0 no action, 1...360 fix heading, neg. = Index to POI in WP List
	 *  @param ToleranceRadius 		in meters, if the MK is within that range around the target, then the next target is triggered
	 *  @param HoldTime 			in seconds, if the was once in the tolerance area around a WP, this time defines the delay before the next WP is triggered
	 *  @param Event_Flag			future implementation
	 *  @param Index				to indentify different waypoints, workaround for bad communications PC <-> NC; start from 1!
	 *  @param Value				type of Waypoint
	 *  @param WP_EventChannelValue	will be transferred to the FC and can be used as Poti value there
	 *  @param AltitudeRate			rate to change the setpoint
	 *  @param Speed				rate to change the Position
	 *  @param CamAngle				Camera servo angle
	 * **/
	public void _sendTarget(int Longitude, int Latitude, int Altitude, int Heading, int ToleranceRadius, int HoldTime, int Event_Flag, int Index, int Type, int WP_EventChannelValue, int AltitudeRate,int Speed, int CamAngle){
 		System.out.println("<SERIALWRITER>sendTarget");

 		Waypoint_t wp_target = new Waypoint_t("new target");
 		wp_target.Position.Longitude.value = Longitude; 
 		wp_target.Position.Latitude.value = Latitude;
 		wp_target.Position.Altitude.value = Altitude;
 		wp_target.Position.Status.value = wp_target.Position.NEWDATA;
 		wp_target.Heading.value = Heading; 
 		wp_target.ToleranceRadius.value = ToleranceRadius;
 		wp_target.HoldTime.value = HoldTime; 
 		wp_target.Event_Flag.value = Event_Flag;
 		wp_target.Index.value = Index; 
 		wp_target.Type.value = Type;
 		wp_target.WP_EventChannelValue.value=WP_EventChannelValue;
 		wp_target.AltitudeRate.value = AltitudeRate;
 		wp_target.Speed.value = Speed;
 		wp_target.CamAngle.value = CamAngle;
		
		encoder.send_command(2,'s',wp_target.getAsInt());
	}
	
	/** (NAVI) SEND WAYPOINT 
	 * 
	 *  @param Longitude 			in 1E-7 deg
	 *  @param Latitude 			in 1E-7 deg
	 *  @param Altitude				in mm
	 *  @param Heading 				orientation, 0 no action, 1...360 fix heading, neg. = Index to POI in WP List
	 *  @param ToleranceRadius 		in meters, if the MK is within that range around the target, then the next target is triggered
	 *  @param HoldTime 			in seconds, if the was once in the tolerance area around a WP, this time defines the delay before the next WP is triggered
	 *  @param Event_Flag			future implementation
	 *  @param Index				to indentify different waypoints, workaround for bad communications PC <-> NC; start from 1!
	 *  @param Value				type of Waypoint
	 *  @param WP_EventChannelValue	will be transferred to the FC and can be used as Poti value there
	 *  @param AltitudeRate			rate to change the setpoint
	 *  @param Speed				rate to change the Position
	 *  @param CamAngle				Camera servo angle 
	 * **/
	public void _sendWP(int Longitude, int Latitude, int Altitude, int Heading, int ToleranceRadius, int HoldTime, int Event_Flag, int Index, int Type, int WP_EventChannelValue, int AltitudeRate,int Speed, int CamAngle){
		
    	System.out.println("<SERIALWRITER>sendWP");
    	Waypoint_t wp = new Waypoint_t("new wp");
    	wp.Position.Longitude.value = Longitude; 
 		wp.Position.Latitude.value = Latitude;
 		wp.Position.Altitude.value = Altitude;
 		wp.Position.Status.value = wp.Position.NEWDATA;
 		wp.Heading.value = Heading; 
 		wp.ToleranceRadius.value = ToleranceRadius;
 		wp.HoldTime.value = HoldTime; 
 		wp.Event_Flag.value = Event_Flag;
 		wp.Index.value = Index; 
 		wp.Type.value = Type;
 		wp.WP_EventChannelValue.value=WP_EventChannelValue;
 		wp.AltitudeRate.value = AltitudeRate;
 		wp.Speed.value = Speed;
 		wp.CamAngle.value = CamAngle;
 		
		encoder.send_command(2,'w',wp.getAsInt());
	}
	
	/** (NAVI)REQUEST WP
	 * 	@param Index
	 *  **/
	public void _reqWP(int Index){
    	System.out.println("<SERIALWRITER>reqWP");
		u8 WP = new u8("New WP");
		WP.value = Index;	//0 for index is invalid; count starts at 1	
		encoder.send_command(2,'x',WP.getAsInt());
	}
	
	/** (NAVI) SERIAL LINK TEST 
	 * 	@param ID going to be echoed back
	 * **/
	public void _serialTest(int ID){	
    	System.out.println("<SERIALWRITER>serialTest");
		u16 echo = new u16("echo");
		echo.value=ID;
		encoder.send_command(2,'z',echo.getAsInt());	
	}
	
	/** (NAVI) SYSTEM TIME INTERVAL
	 * 	@param intervalST in 10ms (ABO will end after 8 seconds)
	 *  **/
	public void _systemTimeInterval(int intervalST){
    	System.out.println("<SERIALWRITER>System time Interval");
    	u8 interval_time = new u8("interval");
    	interval_time.value = intervalST; //PASSING 0 STOPS THE STREAM
		encoder.send_command(2,'t',interval_time.getAsInt());
		subscription[2] = intervalST;
	}
	
	/** (NAVI) SET 3D DATA INTERVAL
	 * 	@param interval3D in 10ms (ABO will end after 8 seconds)
	 * **/
	public void _3DDataInterval(int interval3D){	
    	System.out.println("<SERIALWRITER>3DDataInterval (in steps of 10 ms)");
		u8 interval_3D = new u8("interval");
		interval_3D.value = interval3D; //PASSING 0 STOPS THE STREAM
		encoder.send_command(2,'c',interval_3D.getAsInt());
		subscription[3] = interval3D;
	}
	
	/** (NAVI) SET OSD DATA INTERVAL
	 * 	@param interval0SD in 10ms (ABO will end after 8 seconds)
	 * **/
	public void _OSDDataInterval (int intervalOSD){
    	System.out.println("<SERIALWRITER>OSDDataInterval (in steps of 10ms)");
		u8 interval_OSD = new u8("interval");
		//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
		interval_OSD.value = Long.valueOf(50).longValue();
		encoder.send_command(2,'o',interval_OSD.getAsInt());
		subscription[4] = intervalOSD;

	}
	
	/** (COMON) REQUEST DISPLAY INTERVAL
	 * 	@param remoteKey		?? Display line; open ticket on github
	 * 	@param intervalDis		in 10ms (ABO will end after 8 seconds)
	 * **/
	public void _ReqDisplay(int remoteKey, int intervalDis){		
		System.out.println("<SERIALWRITER>Req Display (Line?)");	 
		u8 key = new u8("line"); //what is this remote key? Display Line?
		key.value = remoteKey;
		
		//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
		u8 intervalDisplay = new u8("interval");
		intervalDisplay.value = intervalDis;	    	
		encoder.send_command(2,'h',concatArray(key.getAsInt(), intervalDisplay.getAsInt()));
	}
	
	
	/** (FLIGHT) ENGINE TEST
	 * 	@param 	duration		time (in ms) during which the motors run; this might not be true: as firmware probably will test every motor during 240ms whatever the value entered
	 * 	@param	amountOfMotors	how much motors does the copter have?
	 * **/
	public void _EngineTest(int duration, int amountOfMotors){
    	System.out.println("<SERIALWRITER>Redirecting Uart to Flight Ctrl");	    	
	 	//param: 1 byte 0x00 for FC; OxO1 for MK3MAG; 0x02 for MKGPS according to http://www.mikrokopter.de/ucwiki/en/SerialProtocol
	 	int[] param_engine = new int[0];
	 	param_engine[0] = 0x00;
	 	encoder.send_command(2,'u',param_engine);

	 	//TODO wait (add delay).. how long? => ticket on github
    	System.out.println("<SERIALWRITER>EngineTest");

		/** Actual Engine Test; no longer then 10 just as a precaution; could be replaced by future min & max system (on engine_t)**/
		int val = 0;
		String data = "10";
		
		if (duration > 10 || duration < 0 ){
			val = 10;
		} else {
			val = Integer.parseInt(data);
		}
		
		Engine_t engines = new Engine_t();
		for (int i=0; i<amountOfMotors; i++){	
			engines.setValue(i,duration);
		}
		
		encoder.send_command(1,'t',engines.getAsInt());

		//TODO again wait (add delay)
		//send magic packet to switch back to navictrl
		encoder.send_magic_packet();
	}
	
	/** This will return a frame in the int[] format for testing purposes; it behaves as a frame coming from the MK **/
	public int[] makeACKFrame(char cmd){
		
		return null;
	}
	

	/** NOTE: 
	 * 	Also atm external control/ftp/set&getParam has not been implemented
	 * TODO check http://forum.mikrokopter.de/topic-43135.html
	 * **/
	public void sendTestCommand(String frameType){	
		 switch (frameType) {
		 	
		 	case "ftpCmd":
		 		/** (NAVI) FTP**/
		    	System.out.println("<SERIALWRITER>FTP (not yet implemented)");
		    	System.out.println("Disabled!");
		    	
		 		/** NOTE: This functionality is not yet documented inside the serial command list; I noted it in the firmware however & it has been mentioned
		 		 * on the firmware update page also.
		 		 * **/
		 		
		 		//TODO Implement FTP feature
			 break;
			 
		 	case "externControll":
		 		/** (NAVI) EXTERNAL CONTROLL**/
		    	System.out.println("<SERIALWRITER>External Ctrl (not yet implemented)");
		    	System.out.println("Disabled!");
		    	
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
		    	
		    	//TODO Implement
		 		break;
		 		
		 	case "setParam" : 
		 		/** (NAVI) SET/GET NC PARAMETER **/
		 		System.out.println("<SERIALWRITER>Set Param");
		 		System.out.println("Disabled!");
		 		//TODO implement feature
		 		//this is DISABLED; changing parameters while in operation could cause MK to CRASH!
		 		//if you want to implement this: create a special type
		 		//u8 get(0)/set(1)
		 		//u8 parameterID
		 		//s16 value
		 		//parse it to an int[]
		 		//encoder.send_command(2,'j',???);
		 		break;
		 		
		    case "sendWPlist":
		    	/** (NAVI) SEND WAYPOINT LIST **/
		    	//TODO 
		    	System.out.println("<SERIALWRITER>sendWPlist");
		    	System.out.println("Disabled!");
		    	break;
		    	
		    	
		    	
		    	
		 
		 	case "version":	
		 		_version();
			 break;

		    case "DebugReqName":
		    	int defaultIndex = 0;
		    	_DebugReqName(defaultIndex);
		    	break;
		    	
		    case "DebugReqDataInterval":
		    	//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
		    	int defaultFrameInterval_DB=50;
		    	_DebugReqDataInterval(defaultFrameInterval_DB);
		    	break;
			 
		 	case "BLStatusInterval":
		 		//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
		    	int defaultFrameInterval_BL=50;
		    	_BLStatusInterval(defaultFrameInterval_BL);		 		
		 		break;

		 	case "redirectUART" :
		 		System.out.println("<SERIALWRITER>Redirect Uart");
		 		System.out.println("Disabled!");		 		
			 	break;
		 
		 	case "errorText":
		 		_errorText();		 		
		 		break;
			 
		 	case "sendTarget":
		 		/** example of a valid wp **/
		 		 
		 		//latitude	 			36827906
		 		//longitude 			510481055
		 		//altitude				5038
		 		//heading				0
		 		//toleranceradius		10
		 		//holdtime				2
		 		//eventflag				1
		 		//index					1
		 		//value					POINT_TYPE_WP
		 		//wp_eventchannelvalue	100
		 		//altituderate			30
		 		//speed					30
		 		//camangle				0
		 		
		 		_sendTarget(36827906,510481055,5038,0,10,2,1,1,POINT_TYPE_WP,100,30,30,0);
		 		
		 		break;
		 
		    case "sendWP":
		    	//We define make a waypoint with some default data for testing		    	
		    	//latitude	 			36827906
		 		//longitude 			510481055
		 		//altitude				5038
		 		//heading				0
		 		//toleranceradius		10
		 		//holdtime				2
		 		//eventflag				1
		 		//index					1
		 		//value					POINT_TYPE_WP
		 		//wp_eventchannelvalue	100
		 		//altituderate			30
		 		//speed					30
		 		//camangle				0
	    	
	    		_sendWP(36827906,510481055,5038,0,10,2,1,1,POINT_TYPE_WP,100,30,30,0);
		    	break;
		    		    	
		    case "reqWP":
				//Get WP 0
		    	_reqWP(1);
		    	break;
		    	
		    case "serialTest":
		    	//we send number 169
		    	_serialTest(169);
		    	break;
		    	
		    case "systemTimeInterval" :		
				//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
		    	int intervalST = 50;
		    	_systemTimeInterval(intervalST);
		    	break;
		    	
		    case "3DDataInterval":		    	
		    	//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
		    	int interval3D = 50;
		    	_3DDataInterval(interval3D);
		    	break;
		    	
		    case "OSDDataInterval":
		    	//we subscribe for a 0.5 second interval (50*10); ABO will end after 8 seconds
		    	int intervalOSD = 50;
		    	_OSDDataInterval(intervalOSD);	    	
		    	break;
		    	
		    case "ReqDisplay" :
		    	int remoteKey = 1;
		    	int intervalDisplay = 50;
		    	_ReqDisplay(remoteKey, intervalDisplay);
		    	break;
		    	
		    case "EngineTest":
		    	_EngineTest(10, 6); 
	    		
		    	break;

		    	
	    }
	}
	
	Timer timer; 
	//start timer
	public void startSubscriptionService(){
		
		timer = new Timer();
		timer.scheduleAtFixedRate(new TimerTask() {
			  public void run() {
				//loop over the array and unless time is zero resend frame
					
					 /** Subscription Service
				     * 
				     * 	0 => DebugReqDataInterval
				     *  1 => BLStatusInterval
				     *  2 => SystemTimeInterval
				     *  3 => 3DDataInterval
				     *  4 => OSDInterval
				    **/
					
					for (int i=0; i<subscription.length; i++){
						if (subscription[i] == 0){
							//do nothing
						} else {
							//resend frame
							if (i==0){
								_DebugReqDataInterval(subscription[i]);
							} else if (i==1){
								_BLStatusInterval(subscription[i]);
							} else if (i==2){
								_systemTimeInterval(subscription[i]);
							} else if (i==3){
								_3DDataInterval(subscription[i]);
							} else if (i==4){
								_OSDDataInterval(subscription[i]);
							}
						}
					}
			  }
			}, 6000,6000);
		
	}
	
	//stop timer
	public void stopSubscriptionService(){
		timer.cancel();
		
	}
	
	
	
	 public static int[] concatArray(int[] a, int[] b) {
	        if (a.length == 0) {
	            return b;
	        }
	        if (b.length == 0) {
	            return a;
	        }
	        int[] ret = new int[a.length + b.length];
	        for (int i = 0; i < a.length; i++) {
	            ret[i] = a[i];
	        }
	        for (int i = a.length; i < a.length + b.length; i++) {
	            ret[i] = b[i - a.length];
	        }
	        return ret;
	    }
}
