/** 
 * @author Thomas Verbeke
 * 
 * This class reads and decodes the frames it reads on the given serial port.
 * It puts the contents in a BloackingQueue which is send to the front-end.
 * 
 * TODO DataStorage class needs to be removed in the long term; maybe keep it for testing only but develop own solution.
 * **/


//TODO Move decoding to another function; write a test class for it; encode data; 

package communication;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import gnu.io.*;
//import javax.comm.*; Doesn't work with javax.com

import datatypes.*;

public class SerialReader extends CommunicationBase implements Runnable,SerialPortEventListener {
	
	static CommPortIdentifier portId;
	static CommPortIdentifier saveportId;
	SerialPort serialPort;
	Thread readThread;
	//static boolean outputBufferEmptyFlag = false;
	boolean isUSB;
	static HashMap<String, CommPortIdentifier> portMap;
	private BlockingQueue<ArrayList> readQueue = new LinkedBlockingQueue<ArrayList>();
	ArrayList<String> list = new ArrayList<String>();
	//Encode encoder;
	BlockingQueue<ArrayList> writeQueue = new LinkedBlockingQueue<ArrayList>();
	 
	//map our CommPorts
	public static HashMap<String, CommPortIdentifier> getPorts() {
        if (portMap == null) {
            portMap = new HashMap<String, CommPortIdentifier>();
            Enumeration portList = CommPortIdentifier.getPortIdentifiers();
            System.out.println("Printing ports:");
           
            while (portList.hasMoreElements()) {
                portId = (CommPortIdentifier) portList.nextElement();     
                if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
                    portMap.put(portId.getName(), portId);
                    System.out.println("-portName: " + portId.getName() + "   -portID: " + portId);
                }
            }
        }
        return portMap;
    }


        
	//constructor set data
	public SerialReader(String port) throws Exception {
			getPorts();
			isUSB =false; //TODO change isUSB implementation?
			
			//TODO Check for correct implementation default port
			String defaultPort;
			 
			// determine the name of the serial port on several operating systems
	        //set defaultPort
	        String osname = System.getProperty("os.name", "").toLowerCase();
	        if (osname.startsWith("windows")) {
	            // windows
	            defaultPort = "COM1";
	        } else if (osname.startsWith("linux")) {
	            // linux
	            defaultPort = "/dev/ttyS0";
	        } else if (osname.startsWith("mac")) {
	            // mac
	            defaultPort = "/dev/cu.usbserial-*";// "/dev/cu.usbserial-* >> SHOULD BE REPLACED!!
	        } else {
	            System.out.println("Sorry, your operating system is not supported");
	            return;
	        }

	        //no port set -> take default port
	        if (port != null) {
	            defaultPort = port;
	        }


	        System.out.println("Set default port to " + defaultPort);

	        // parse ports and if the default port is found, initialized the reader
	       
	        if (!portMap.keySet().contains(defaultPort)) {
	            System.out.println("port " + defaultPort + " not found.");
	            System.out.println("Exiting the program now...");
	            System.exit(0);
	      
	        } else {  	
	        	 portId = portMap.get(defaultPort);
	             
	             //open the port with: Name, TimeOut (timeout in msec)
	             serialPort = (SerialPort) portId.open("SimpleReadApp", 2000);
	             inputStream = serialPort.getInputStream();

	             serialPort.addEventListener(this);

	             // activate the DATA_AVAILABLE notifier
	             serialPort.notifyOnDataAvailable(true);

	             // set port parameters
	             serialPort.setSerialPortParams(57600, SerialPort.DATABITS_8,
	                     SerialPort.STOPBITS_1,
	                     SerialPort.PARITY_NONE);

	             //encoder = new Encode(serialPort.getOutputStream());
	             // first thing in the thread, we initialize the write operation
	             initwritetoport();  
	             
	             //start a new blocking thread that's going to read or blocking output buffer
	             
	             /**
	             Thread thread = new Thread(new SerialWriter(serialPort.getOutputStream(),event));
	     		 thread.start();
	             **/
	        }     
	        
	        //TODO Update this; we need to put in our DataStorage class
			
		
		
		//String data = "New data has arrived";
		//queue.add(data);
		// start the read thread
        readThread = new Thread(this);
        readThread.start(); 
        
        //TODO Re-add when testing with serial port enabled
        //encoder.send_command(0,'o',20);
		
	}

	public void run() {
		// Communicate with serial port; store data in class
		//TODO there was a timer here; check it
		
		while (true) {
            try {
           
            	Thread.sleep(3000); 
            	//test();
            	
            	
            	//demo
            	
            	//int interval = 100;     //multiplied by 10 and then used as miliseconds -> 1 second; Subsciption needs to be renewed every 4s
                //encoder.send_command(0,'d',interval);
                
       
                
            	//int num = (int) (Math.random() * 100);
    			//queue.add("Data: "+num); //BOGUS DATA
	
                //encoder.send_command(0,'o',20);           	
         		//TODO We should only replace the attribute when data is new; so we should have a way of knowing if data is new or not
         	
         		
            } catch (InterruptedException e) {
            	System.out.println("Interrupted Exception");
            }
        }
	}

	//former commbase code 
	int MAX_SIZE_BUFFER = 190;
	byte COMBuffer[] = new byte[MAX_SIZE_BUFFER];
	int buf_ptr=0;
	int crc, crc1,crc2;
	char UartState=0;
	boolean CrcOkay = false;
	byte[] readBuffer = new byte[220];
	volatile int RxdBuffer[] = new int[MAX_SIZE_BUFFER];
	//volatile boolean NewDataRecieved = false;
	int recievingBytes = 0;
	 
	public void serialEvent(SerialPortEvent event) {
		
	        //System.out.println("SERIAL EVENT");
	        switch (event.getEventType()) {
	            //http://docs.oracle.com/cd/E17802_01/products/products/javacomm/reference/api/javax/comm/SerialPortEvent.html
	            case SerialPortEvent.BI:    //break interrupt
	            case SerialPortEvent.OE:    //overrun error
	            case SerialPortEvent.FE:    //framing error
	            case SerialPortEvent.PE:    //parity error
	            case SerialPortEvent.CD:    //carrier detect
	            case SerialPortEvent.CTS:   //clear to send
	            case SerialPortEvent.DSR:   //data set ready
	            case SerialPortEvent.RI:    //ring indicator
	            case SerialPortEvent.OUTPUT_BUFFER_EMPTY:   //output buffer is empty
	               break;
	            case SerialPortEvent.DATA_AVAILABLE:        //data available at serial port
	                //System.out.println("Data Available at serial port");
	                try {
	                    while (inputStream.available() > 0) {
	                        int numBytes = inputStream.read(readBuffer);
	                        int foo = 0;
	                        //TODO remove magic pakket handler (+ check if can be removed)
	                        /** SELF: WHAT IS HAPPENING HERE? MAGIC PACKET
	                         *  http://forum.mikrokopter.de/topic-14090.html
	                         *  Magic packet to switch to navi-ctrl
	                         *  Should not matter; could be removed; code is emulating MK firmware
	                         * 
	                         **/
	                        //"0x1B,0x1B,0x55,0xAA,0x00"
	                        while (foo < readBuffer.length - 5
	                                && (readBuffer[foo] != 0x1B
	                                || readBuffer[foo + 1] != 0x1B
	                                || readBuffer[foo + 2] != 0x55
	                                //|| readBuffer[foo + 3] != 0xAA
	                                || readBuffer[foo + 4] != 0x00)) {
	                            foo++;
	                        }
	                        if (readBuffer[foo] == 0x1B
	                                && readBuffer[foo + 1] == 0x1B
	                                //&& readBuffer[foo + 2] != 0x55
	                                && readBuffer[foo + 3] != 0xAA //&& readBuffer[foo + 4] != 0x00
	                                ) {
	                            //DataStorage.setUART(DataStorage.UART_CONNECTION.NC);
	                            UartState = 0;
	                        } else {
	                           
	                            for (int i=0; i<numBytes; i++) {
	                            	UART_vect((char) readBuffer[i]);
	                            }
	                        }
	                    }
	                } catch (IOException ex) {
	                }
	                break;
	        }
	    }
	
	private void UART_vect(char SerialCharacter) {
    	//System.out.println("inside loop");
       
    	//System.out.println("buffer_ "+SerialCharacter);
    	//System.out.println("state_"+UartState);
    	
    	//overflow check
    	if (buf_ptr >= MAX_SIZE_BUFFER){
    		UartState = 0;
    		System.out.println("Buffer overflow");
    	}
    	
    	//end of frame
    	if (SerialCharacter == '\r' && UartState ==2){
    		//System.out.println("end of frame");
    		UartState = 0; //reset Uart State
    		//CRC check
    		crc -= RxdBuffer[buf_ptr-2];
    		crc -= RxdBuffer[buf_ptr-1];
    		crc %= 4096;
    		crc1 = '=' + crc / 64;
            crc2 = '=' + crc % 64;
            CrcOkay = false;
            if ((crc1 == RxdBuffer[buf_ptr - 2]) && (crc2 == RxdBuffer[buf_ptr - 1])) {
                CrcOkay = true;
                //System.out.println("CRC check OK");
            } else {
                CrcOkay = false;
                System.out.println("CRC check failed");
            }
            
            //if crc check is true; start decoding command
            if (CrcOkay){
            //if (!NewDataRecieved && CrcOkay){
            	//NewDataRecieved = true;
            	recievingBytes = buf_ptr + 1;
            	RxdBuffer[buf_ptr] = SerialCharacter; //or RxdBuffer[buf_ptr] = '\r'
            	buf_ptr = -1;
            	
            	final int RxdBuffer_work[] = new int[MAX_SIZE_BUFFER];
            	System.arraycopy(RxdBuffer, 0, RxdBuffer_work, 0, RxdBuffer.length);
            	
//            	DataStorage.executors.submit(new Runnable() {
//                    public void run() {
//                    	//System.out.println("start decoding in a new thread");           	
//                    	decode_buffer(RxdBuffer_work);
//                    }
//                }); 
            	
            	Thread thread = new Thread(new Runnable(){

					@Override
					public void run() {
						// TODO Auto-generated method stub
						decode_buffer(RxdBuffer_work);
					}
            		
            	});
	     		thread.start();
            }	                               	                                   	                                                                        
    	} else {
    		//walk trough frame
    		//System.out.println("Else: "+NewDataRecieved);
    		if (SerialCharacter == '#'){
				//System.out.println("Start delimeter detected status: " + NewDataRecieved);
			}	
    		switch(UartState){
        		case 0:
        			if (SerialCharacter == '#'){
        			//if (SerialCharacter == '#' && !NewDataRecieved){
        				//start delimiter detected
        				//System.out.println("Start delimter detected + NewDataRecieved is false");
        				UartState = 1;
        			}	   
        			
        			 buf_ptr = 0;//reset buf_ptr
                     RxdBuffer[buf_ptr] = SerialCharacter;
                     crc = SerialCharacter;
                     buf_ptr++;	 
                     break;
        		case 1:
        			//address
        			//System.out.println("Uart state : 1");
        			UartState++;
        			RxdBuffer[buf_ptr] = SerialCharacter;
        			crc +=SerialCharacter;
        			buf_ptr++;
        			break;
        		case 2:
        			//data
        			//System.out.println("Uart state : 2");
        			RxdBuffer[buf_ptr] = SerialCharacter;
        			if (buf_ptr < MAX_SIZE_BUFFER){
        				buf_ptr++;
        			} else {
        				UartState = 0;
                		System.out.println("Buffer overflow");
        			}
        			crc +=SerialCharacter;
        			break;
        		default:
        			UartState = 0;
        			break;
    		}
    	}
    }

	int numBytesDecoded = 0;
    int dataPointer = 0;
    
    /** 
     * @author Claas Anders "CaScAdE" Rathje
     * @author Marcus -LiGi- Bueschleb
     * http://github.com/ligi/DUBwise/blob/master/shared/src/org/ligi/ufo/MKCommunicator.java
     * **/
	public int[] Decode64(int[] RxdBuffer) {
       
		/** --Protocol-- (http://www.mikrokopter.de/ucwiki/en/SerialProtocol)
        * 	Start-Byte: 			'#'
        * 	Address Byte: 			'a'+ Addr
        * 	ID-Byte:				'V','D' etc'
        * 	n Data-Bytes coded: 	"modified-base64"
        * 	CRC-Byte1:				variable
        * 	CRC-Byte2:				variable
        * 	Stop-Byte:				'\r'
        */
		
		int a, b, c, d;
        int x, y, z;
        int ptrIn = 3; // start at begin of data block
        int ptrOut = 3;
        int len = recievingBytes - 6; // of the total byte of a frame remove: 3 bytes from the header ('#', Addr, Cmd) and 3 bytes from the footer (CRC1, CRC2, '\ r')
        //TODO don't forget to to the same when looping over recieving bytes we do not need to loop over CRC1,CRC2,..

        while (len != 0) {
            a = 0;
            b = 0;
            c = 0;
            d = 0;
            try {
                a = RxdBuffer[ptrIn++]	 - '=';
                b = RxdBuffer[ptrIn++] - '=';
                c = RxdBuffer[ptrIn++] - '=';
                d = RxdBuffer[ptrIn++] - '=';
            } catch (Exception e) {
            }

            x = (a << 2) | (b >> 4);
            y = ((b & 0x0f) << 4) | (c >> 2);
            z = ((c & 0x03) << 6) | d;

            if ((len--) != 0) {
                RxdBuffer[ptrOut++] = x;
            } else {
                break;
            }
            if ((len--) != 0) {
                RxdBuffer[ptrOut++] = y;
            } else {
                break;
            }
            if ((len--) != 0) {
                RxdBuffer[ptrOut++] = z;
            } else {
                break;
            }
        }
        dataPointer = 3; // decoded data starts from byte 4 
        numBytesDecoded = ptrOut - 3;  // how much bytes were decoded?
        
        return RxdBuffer;

    }
	
	public static int[] analog = new int[32];
    public static String[] names = new String[32];
    public static int name_counter = 0;
    
	private void decode_buffer(int[] dataFrame) {
		//TODO implement adding to the queue for each element
		String data = "New data has arrived";
		//queue.add(data);
		//context.getServletContext().setAttribute("serialPortData", queue);	
		
		//TODO Some of these values need to be remembered outside of frame handling (motorList, motorCount)
        u8 WPIndex;
        u8 numberOfWP;
        u8 parameterId;
        s16 parameterValue;
        String displayMessage;
        String label;
        
        BLData_t motorList[] = new BLData_t[5]; //6 motors (0->5)
        int motorCount = 0;
        
        /**
		if (!NewDataRecieved){
			System.out.println("NewDataRecieved (inside decode_buffer): " + NewDataRecieved );
			return;
		}
		
			**/
		
		/** --Protocol-- (http://www.mikrokopter.de/ucwiki/en/SerialProtocol)
	        * 	Start-Byte: 			'#'
	        * 	Address Byte: 			'a'+ Addr
	        * 	ID-Byte:				'V','D' etc'
	        * 	n Data-Bytes coded: 	"modified-base64"
	        * 	CRC-Byte1:				variable
	        * 	CRC-Byte2:				variable
	        * 	Stop-Byte:				'\r'
	     */
		
		int[] decodedDataFrame = Decode64(dataFrame); //decode the frame; 
		
		 /**  
         * REMARK!
         * 
         * FC address = 'a' + 1 -> ascii 'b' or decimal 98 
         * NC address = 'a' + 2 -> ascii 'c' or decimal 99
         * 
         **/
		
		decodedDataFrame[1] = decodedDataFrame[1] - 'a';  //removing 'a'
		
		switch (decodedDataFrame[1]){
			case NC_ADDRESS:		//frame from Navi Ctrl
				switch (decodedDataFrame[2]){
					//NAVI-CTRL
		            case 'Z':   // Serial link test
		                System.out.println("<Z>Serial Link Test");
		                
		                u16 _echoPattern = new u16("loaded");
		                _echoPattern.loadFromInt(decodedDataFrame, dataPointer);
		                
		                System.out.println("<Z> Returns: EchoPattern");
		                //System.out.println("u16: " + _echoPattern.value);
		                
		                
		                ArrayList serialLinkTest = new ArrayList();                 
		                serialLinkTest.add("serialTest");	//set ID
		                serialLinkTest.add(_echoPattern.value);
		                readQueue.add(serialLinkTest);	
		                
		                break;    
		                            
		            case 'E':   // Feedback from Error Text Request; Error Str from Navi
		                System.out.println("<E> Returns: Error Str from NaviCtrl");
		                
		                char errorMessage= (char) decodedDataFrame[dataPointer];
		                System.out.println(errorMessage);
		                
		                ArrayList errorMsg = new ArrayList();                 
		                errorMsg.add("errorMsg");	//set ID
		                errorMsg.add(errorMessage);
		                readQueue.add(errorMsg);	                
		                break;
		                            
		            case 'W':   // Feedback from 'send WP' command
		                System.out.println("<W> Feedback from 'send WP' command - # Waypoints in memory");
		                //System.out.println("<W> Returns: # Waypoints in memory");
		                //WORKS but there are some more values inside the array?
		                //http://forum.mikrokopter.de/topic-post441164.html#post441164
		                //DOES NOT GIVE NUMBER OF WP BUT THE INDEX OF THE LAST WP??
		                	                
		                WPIndex = new u8("WPIndex");
		                WPIndex.loadFromInt(decodedDataFrame, dataPointer);
		                //System.out.println("WPIndex (u8): " + WPIndex.value); 
		                ArrayList sendWP = new ArrayList(); 
		                
		                sendWP.add("sendWP_ACK");	//set ID
		                sendWP.add(WPIndex.value); //number of WP
		                readQueue.add(sendWP);
		                
		                break;
		                
		            case 'X':   // Feedback from 'request WP' command
		                System.out.println("<X> Returns: # Waypoints in memory; WP index; WP struct");	                
		                System.out.println("<Debug> Number of bytes decoded: "+numBytesDecoded); //debug
		                
		                /** -- Data Structure--
		                 * 
		                 * byte 1: u8 number of WP
		                 * byte 2: u8 WP index
		                 * from byte 3: Waypoint struct
		                */
		          	               
		                numberOfWP = new u8("numberOfWP");
		                numberOfWP.loadFromInt(decodedDataFrame, dataPointer);
		                //System.out.println("NymberOfWP (u8): " + numberOfWP.value);
		                
		                WPIndex = new u8("WPIndex");
		                WPIndex.loadFromInt(decodedDataFrame, dataPointer+1);
		                //System.out.println("WPIndex (u8): " + WPIndex.value);
		                
		             
		                Waypoint_t WP= new Waypoint_t("WP");
		                WP.loadFromInt(decodedDataFrame, dataPointer+2);
		                
	                	ArrayList reqWP = new ArrayList(); 	
	                	
	                	reqWP.add("reqWP");	//set ID
	                	reqWP.add(numberOfWP.value); //number of WP
	                	reqWP.add(WPIndex.value); //WP index
	                	
	                	reqWP.add(WP.Position.Latitude.value); 
	                	reqWP.add(WP.Position.Longitude.value); 
	                	reqWP.add(WP.Position.Altitude.value); 
	                	reqWP.add(WP.Position.Status.value); 
	                	
	                	reqWP.add(WP.Heading.value);
	                	reqWP.add(WP.ToleranceRadius.value);
	                	reqWP.add(WP.HoldTime.value);
	                	reqWP.add(WP.Event_Flag.value);
	                	reqWP.add(WP.Index.value);
	                	reqWP.add(WP.Type.value);
	                	reqWP.add(WP.WP_EventChannelValue.value);
	                	reqWP.add(WP.AltitudeRate.value);               	
	                	reqWP.add(WP.Speed.value);
	                	reqWP.add(WP.CamAngle.value);
	                	reqWP.add(WP.name.toString());
	
		                readQueue.add(reqWP);
		                              
		                break;    
	
		            case 'O':   // Feedback from 'request OSD Values'
		                //System.out.println("<O> Recieving OSD Data");
		                
		                NaviData_t navi = new NaviData_t();
		                navi.loadFromInt(decodedDataFrame,dataPointer); 
		                //navi.HomePosition.printOut();
		                //navi.CurrentPosition.printOut();
		               
		                //TODO send this data to front end
		                //build up object
		                ArrayList OSD = new ArrayList(); 
		                
		                /** CurrentPosition **/
		                OSD.add("OSD");
		              
		                OSD.add(navi.CurrentPosition.Latitude.value); // in 1E-7 deg
		                OSD.add(navi.CurrentPosition.Longitude.value); // in 1E-7 deg
		                OSD.add(navi.CurrentPosition.Altitude.value); // in mm
		                OSD.add(navi.CurrentPosition.Status.value); // validity of data {INVALID,NEWDATA,PROCESSED}
		                /** TargetPosition **/
		                OSD.add(navi.TargetPosition.Latitude.value); // in 1E-7 deg
		                OSD.add(navi.TargetPosition.Longitude.value); // in 1E-7 deg
		                OSD.add(navi.TargetPosition.Altitude.value); // in mm
		                OSD.add(navi.TargetPosition.Status.value); // validity of data {INVALID,NEWDATA,PROCESSED}
		                /** HomePosition **/
		                OSD.add(navi.HomePosition.Latitude.value); // in 1E-7 deg
		                OSD.add(navi.HomePosition.Longitude.value); // in 1E-7 deg
		                OSD.add(navi.HomePosition.Altitude.value); // in mm
		                OSD.add(navi.HomePosition.Status.value); // validity of data {INVALID,NEWDATA,PROCESSED}
		              
		                OSD.add(navi.WaypointIndex.value);  // index of current waypoints running from 0 to WaypointNumber-1
		                OSD.add(navi.WaypointNumber.value); // number of stored waypoints
		                OSD.add(navi.SatsInUse.value);   // number of satellites used for position solution
		                OSD.add(navi.Altimeter.value);	 // hight according to air pressure
		                OSD.add(navi.Variometer.value);	 // climb(+) and sink(-) rate
		                OSD.add(navi.FlyingTime.value);	// FlyingTime in seconds
		                OSD.add(navi.UBat.value);	// Battery Voltage in 0.1 Volts
		                OSD.add(navi.GroundSpeed.value); // speed over ground in cm/s (2D)
		                OSD.add(navi.Heading.value); // current flight direction in deg as angle to north
		                OSD.add(navi.CompassHeading.value);  // current compass value in deg
		                OSD.add(navi.AngleNick.value);// current Nick angle in 1 deg
		                OSD.add(navi.AngleRoll.value);  // current Rick angle in 1deg
		                OSD.add(navi.RC_Quality.value);	// RC_Quality
		               
		                	                
		                // ------- FCStatusFlags -------------------------------
		                //FC_STATUS_MOTOR_RUN                     0x01
		                //FC_STATUS_FLY                           0x02
		                //FC_STATUS_CALIBRATE                     0x04
		                //FC_STATUS_START                         0x08
		                //FC_STATUS_EMERGENCY_LANDING             0x10
		                //FC_STATUS_LOWBAT                        0x20
		                //FC_STATUS_VARIO_TRIM_UP                 0x40
		                //FC_STATUS_VARIO_TRIM_DOWN               0x80
		                
		                OSD.add(navi.FCFlags.value); // Flags from FC
		                
		                // ------- NCFlags -------------------------------------
		                // NC_FLAG_FREE                            0x01
		                // NC_FLAG_PH                              0x02
		                // NC_FLAG_CH                              0x04
		                // NC_FLAG_RANGE_LIMIT                     0x08
		                // NC_FLAG_NOSERIALLINK                    0x10
		                // NC_FLAG_TARGET_REACHED                  0x20
		                // NC_FLAG_MANUAL                          0x40
		                // NC_FLAG_GPS_OK                          0x80
		                
		                OSD.add(navi.NCFlags.value); // Flags from NC		                
		                OSD.add(navi.Errorcode.value); // 0 --> okay
		                OSD.add(navi.OperatingRadius.value); // current operation radius around the Home Position in m
		                OSD.add(navi.TopSpeed.value);   // velocity in vertical direction in cm/s
		                OSD.add(navi.TargetHoldTime.value); // time in s to stay at the given target, counts down to 0 if target has 
		               
		                // ------- FCStatusFlags2 ------------------------------
		                //FC_STATUS2_CAREFREE_ACTIVE              0x01
		                //FC_STATUS2_ALTITUDE_CONTROL_ACTIVE      0x02
		                //FC_STATUS2_FAILSAFE_ACTIVE              0x04
		                //FC_STATUS2_OUT1                         0x08
		                //FC_STATUS2_OUT2                         0x10
		                //FC_STATUS2_RES1                         0x20
		                //FC_STATUS2_RES2                         0x40
		                //FC_STATUS2_RES3                         0x80
		                
		                OSD.add(navi.FCStatusFlags2.value); // StatusFlags2 (since version 5 added)
		                OSD.add(navi.SetpointAltitude.value); // setpoint for altitude
		                OSD.add(navi.Current.value); // actual current in 0.1A steps
		                OSD.add(navi.Gas.value);
		                OSD.add(navi.UsedCapacity.value); // used capacity in mAh
		                OSD.add(navi.Version.value);
		                readQueue.add(OSD);
		                                
		                break;
		                
		            case 'C':   // Feedback from 'set 3D data interval'
		                System.out.println("<C> Recieving 3D Data");
       
		                Data3D_t data3D = new Data3D_t();
		                data3D.loadFromInt(decodedDataFrame,dataPointer);

		                //System.out.println("<O> Returns: Struct Data3D");
	                	ArrayList set3DInterval = new ArrayList(); 	
	                	
	                	set3DInterval.add("Data3D");	//set ID
	                	set3DInterval.add(data3D.AngleNick.value); // AngleNick in 0.1 deg
		                set3DInterval.add(data3D.AngleRoll.value); // AngleRoll in 0.1 deg
		                set3DInterval.add(data3D.Heading.value); // Heading in 0.1 deg
		                set3DInterval.add(data3D.StickNick.value);  //Stick Nick
		                set3DInterval.add(data3D.StickRoll.value); // Stick Roll
		                set3DInterval.add(data3D.StickYaw.value); // Stick Yaw
		                set3DInterval.add(data3D.StickGas.value); // Stick Gas
		                readQueue.add(set3DInterval);
		                
		                break; 
		            
		            case 'J':   // Feedback from 'Set/Get NC-Parameter' (only set when when there is a value
		                System.out.println("<J> Returns: ParameterId,Value");
		                
		                /** -- Data Structure--
		                 * 
		                 * byte 1: u8 parameterId
		                 * from byte 2: s16 value
		                */
		          	               
		                parameterId = new u8("parameterId");
		                parameterId.loadFromInt(decodedDataFrame, dataPointer);
		                //System.out.println("Parameter Id (u8): " + parameterId.value);
		                
		                parameterValue = new s16("WPIndex");
		                parameterValue.loadFromInt(decodedDataFrame, dataPointer+1);
		                //System.out.println("Parameter Value (s16): " + parameterValue.value);
		             
	                	ArrayList getsetNCParam = new ArrayList(); 	
	                	
	                	getsetNCParam.add("getsetNCParam");	//set ID
	                	getsetNCParam.add(parameterId.value); // parameter ID
	                	getsetNCParam.add(parameterValue.value); //parameter value

		                readQueue.add(getsetNCParam);
		                
		                break;   
		    
		            case 'K':   // Feedback from 'BL Ctrl Status'
	
		                BLData_t motorData = new BLData_t(12);
		                motorData.loadFromInt(decodedDataFrame, dataPointer);
		                //motorData.printOut();
		                
		                System.out.println("<K> Recieving BL Ctrl Data (DC Motor "+motorData.Index.value+" Data)");
		                
		                motorList[motorCount] = motorData;
		                
		                //!! we only get data from 1 motor
		                //TODO Is it the view or the model that needs to take care of this?
		                //store in array
		                if (motorCount != 5){
		                    motorCount++;
		                } else {
		                    motorCount = 0;
		                }
		                
	                	ArrayList BLCtrlStatus = new ArrayList(); 	
	                	
	                	BLCtrlStatus.add("MotorData");	//set ID
	                	BLCtrlStatus.add(motorData.Index.value); //Index
	                	BLCtrlStatus.add(motorData.Current.value);//Current
	                	BLCtrlStatus.add(motorData.Temperature.value);//Temperature
	                	BLCtrlStatus.add(motorData.MaxPWM.value);//MAXPW
	                	BLCtrlStatus.add(motorData.State.value);//Status
	                	//TODO Add status flags from documentation
		                readQueue.add(BLCtrlStatus);		                
		               break; 
					}
				break;
			case FC_ADDRESS:		//frame from Flight Ctrl
				switch (decodedDataFrame[2]){
					case 'T':   // Feedback from 'BL Ctrl Status'
						//!!Empty ack frame
		                System.out.println("<T>Motor Test");
		                ArrayList motorTest = new ArrayList(); 		
		                motorTest.add("motorTest");	//set ID; motorTest frame contains no data    
		                readQueue.add(motorTest);
		                
		                break; 
		                
		            case 'P':   // Feedback from 'BL Ctrl Status'
		            	//TODO Frame not empty; s16 array
		                System.out.println("<P>Read PPM Channels");
		                //queue.add("feedback from PPM Channels");
		                break; 
	
		                
		           case 'N':   // Feedback from 'BL Ctrl Status'
		        	   	//TODO Frame not empty; s16 array
		                System.out.println("<N>Mixer request");
		                //queue.add("Feedback from Mixer request");
		                break; 
		                
		           default: 
		        	   System.out.println("Unsupported command recieved");
		        	   //print command
		        	   System.out.println("Data: 1:"+decodedDataFrame[1] + " 2:"+(char)decodedDataFrame[2]);
		        	   //queue.add("Unsupported command recieved");
		        	   break;
				}
				//break;
				
			default: 				//frame from Common Commands
			
				switch (decodedDataFrame[2]){
					
				    case 'A':   // Feedback from 'get labels of the analog values in debug data struct'		              
			                /** -- Data Structure--
			                 * 
			                 * byte 1: 		u8 Index
			                 * byte 2-end:	char[16] label text
			                */
			                
			                u8 analogLabelIndex = new u8("AnalogLabelIndex");
			            	analogLabelIndex.loadFromInt(decodedDataFrame, dataPointer);
			            	label = "";
		            	
							for (int i=dataPointer+1; i<numBytesDecoded; i++){
								label += (char)decodedDataFrame[i];    
							}
			            	
							//System.out.println("<Debug> Index: "+ analogLabelIndex.value + " Label: "+ label);
							//TODO convert index to int?
			                names[name_counter]+=label;
			                name_counter++; 
			                
			                if (name_counter != 32){
			                	//ask for next label
			                	//TODO may give problems; encoder not implemented yet
			                	//TODO should add better system to ensure good data
			                	//Encode encoder;
								//try {
									//encoder = new Encode(serialPort.getOutputStream());
									//encoder.send_command(0,'a',name_counter);
								//} catch (IOException e) {
									// TODO Auto-generated catch block
								//	System.out.println("Sending serial command failed");
								//	e.printStackTrace();
								//}
			                }
			                
			                System.out.println("<A> Recieving Analog Channel Label" + analogLabelIndex.value);
			                
			                ArrayList analogLabel = new ArrayList(); 		
			                analogLabel.add("analogLabel");	//set ID; 
			                analogLabel.add(analogLabelIndex.value);  
			                analogLabel.add(label);  
			                readQueue.add(analogLabel); //analog label index	
			                break;     
			                
				    case 'B':   // Feedback from 'External Control'
			                System.out.println("<B> Frame: char,echo of Extern ctrl frame as confirmation");
			                ArrayList externCtrl = new ArrayList(); 		
			                externCtrl.add("externCtrl");	//set ID;   
			                //TODO Implement extern ctrl struct: not documented online; look up source code
			                readQueue.add(externCtrl); 
			                //Digital, RemoteTasten, Nick, Roll, Gier, Gas, Hight, Free, Frame, Config
				    		break;
			           
				    case 'H':   // Feedback from 'Request Display/LCD Data'	  1               
			                System.out.println("<H> Recieving Display Data");
			               
			                /** -- Data Structure--
			                 * 
			                 * byte 1-end: char[80] DisplayText
			                */
			                
			                displayMessage="";
			               
			                //iterate trough all the data; start from the dataPointer
			                for (int i=dataPointer; i<numBytesDecoded; i++){
			                   displayMessage += (char)decodedDataFrame[i];    
			                }
			             
			                ArrayList reqDisplay = new ArrayList(); 		
			                reqDisplay.add("reqDisplay");	//set ID; 
			                reqDisplay.add(displayMessage);  		               
			                readQueue.add(reqDisplay); 
			                
			                break;
			                
				    case 'L':   // Feedback from 'Request Display/LCD Data' 2
			                System.out.println("<L> Frame: MenuItem,MaxMenuItem,DisplayText");
		   
			                /** -- Data Structure--
			                 * 
			                 * byte 1: u8 MenuItem
			                 * byte 2: u8 MaxMenuItem
			                 * byte 3: char[80] Display Text
			                */
			                
			                u8 menuItem = new u8("MenuItem");
			                menuItem.loadFromInt(decodedDataFrame, dataPointer);
			                //menuItem.printOut();
			                
			                u8 maxMenuItem = new u8("MaxMenuItem");
			                maxMenuItem.loadFromInt(decodedDataFrame, dataPointer+1);
			                //maxMenuItem.printOut();
			                
			                displayMessage="";
			               
			                //iterate & build up displayMessage
			                for (int i=dataPointer+2; i<numBytesDecoded; i++){
			                   displayMessage = displayMessage+((char)decodedDataFrame[i]);    
			                }
			                  
			                //System.out.println(displayMessage);  
			               
			                ArrayList reqMenuDisplay = new ArrayList(); 		
			                reqMenuDisplay.add("reqMenuDisplay");	//set ID; 
			                reqMenuDisplay.add(menuItem); //MenuItem
			                reqMenuDisplay.add(maxMenuItem); //MaxMenuItem
			                reqMenuDisplay.add(displayMessage); //Display Message
			                
			                readQueue.add(reqMenuDisplay); //analog label index	
			                
			                break;
	
				    case 'V':   // Feedback from 'Version Info'
			                System.out.println("<V> Frame: VersionStruct");
			                
			                VersionInfo_t versionStruct = new VersionInfo_t("versioninfo");
			                versionStruct.loadFromInt(decodedDataFrame, dataPointer);
		   
			                ArrayList versionInfo = new ArrayList(); 		
			                versionInfo.add("versionInfo");	//set ID; 
			                versionInfo.add(versionStruct.SWMajor.value); //SWMajor
			                versionInfo.add(versionStruct.SWMinor.value); //SWMinor
			                versionInfo.add(versionStruct.ProtoMajor.value); //ProtoMajor
			                versionInfo.add(versionStruct.ProtoMinor.value); //ProtoMinor
			                versionInfo.add(versionStruct.SWPatch.value); //SWPatch
			                //TODO: Test this not sure this works
			                //TODO Document hardwareError codes http://www.mikrokopter.de/ucwiki/en/SerialCommands/VersionStruct
			                versionInfo.add(versionStruct.HardwareError.toString()); //HardwareError
			                readQueue.add(versionInfo); 
			                
			                break;
			                
				    case 'D':   // Feedback from 'request debug Data'
			               	System.out.println("<D> Frame: Debug Request");
			               	//TODO check strange param ADRESS
			              
			                /** -- Data Structure--
			                 * 
			                 * unsigned char Status [2]
			                 * signed int Analog[32]
			                */
			               	
			               	char[] status = new char[2];
			               	status[0] = (char) decodedDataFrame[dataPointer];
			               	status[1] = (char) decodedDataFrame[dataPointer+1];
			               	
			               	int[] analog = new int[32];
			               	
			                //skip status and parse unsigned int
			                for (int i=2;i<34;i++){                 
			                	analog[i] = (int) decodedDataFrame[dataPointer+i];
			                }
			               
			                ArrayList analogData = new ArrayList(); 		
			                analogData.add("analogData");	//set ID; 
			                analogData.add(status); 
			                for (int i=0;i<analog.length; i++){
			                	analogData.add(analog);	
			                }

			                readQueue.add(analogData); 
			                
			                break; 
			                
				    case 'G':   // Feedback from 'Get External control'	
			                System.out.println("<G> Returns: ExternControl Struct");
			                //TODO Have to implement this
			                //queue.add("feedback from get external control");
				    		break;
				}
				break;
		}
		
	
	
        dataPointer = 0;
        numBytesDecoded = 0;
       
		
	}


	public void initwritetoport() {
        // initwritetoport() assumes that the port has already been opened and
        // initialized by "public nulltest()"

        try {
            // get the outputstream
            outputStream = serialPort.getOutputStream();
        } catch (IOException e) {
        }

        try {
            // activate the OUTPUT_BUFFER_EMPTY notifier
            // DISABLE for USB
            //System.out.println(serialPort.getName());
            if (!isUSB) {
                serialPort.notifyOnOutputEmpty(true);
            }
        } catch (Exception e) {
            System.out.println("Error setting event notification");
            System.out.println(e.toString());
            System.exit(-1);
        }

    }
	
	public void test(){
	      /** ENGINE TEST**/
        //first redirect UART
    
        //Thread.sleep(1000L);
        //encoder.send_magic_packet();
        //Thread.sleep(1000L);
        
        
        //Waypoint_t newWP = new Waypoint_t("new WP");
        //newWP.Position.Longitude.value = 47008664;
        //newWP.Position.Latitude.value = 508799722;
        //newWP.Position.Altitude.value = 250;
        //newWP.Position.Status.value = 1;
        //newWP.Heading.value = 0;
        //newWP.ToleranceRadius.value = 10;
        //newWP.HoldTime.value = 2;
        //newWP.Event_Flag.value = 0;
        //newWP.Index.value = 1;
        //newWP.Type.value = 0;
        //newWP.WP_EventChannelValue.value=100;
        //newWP.AltitudeRate.value = 30;
        //newWP.Speed.value = 30;
        //newWP.CameraAngle.value = 0;
        //encoder.send_command(2,'w',newWP.getAsInt());
            
        
    	//--WORKS--    
        /** Labels of Analog values request (for all 32 channels) **/
        //encoder.send_command(0,'a',0);
        
        /** (COMMON) DEBUG REQUEST **/
        //interval
        //int interval = 100;     //multiplied by 10 and then used as miliseconds -> 1 second; Subsciption needs to be renewed every 4s
        //encoder.send_command(0,'d',interval);
        
        /** (NAVI) SERIAL LINK TEST **/
        //u16 echo = new u16("echo");
        //echo.value=23;
        //encoder.send_command(2,'z',echo.getAsInt());
        
        /** (NAVI)REQUEST WP **/
        //u8 WP = new u8("New WP");
        //WP.value = 1;
        //encoder.send_command(2,'x',WP.getAsInt());
        
        /** (NAVI) SEND WAYPOINT **/
        //Waypoint_t newWP = new Waypoint_t("new WP");
        //newWP.Position.Longitude.value = 47008664;
        //newWP.Position.Latitude.value = 508799722;
        //newWP.Position.Altitude.value = 250;
        //newWP.Position.Status.value = 1;
        //newWP.Heading.value = 0;
        //newWP.ToleranceRadius.value = 10;
        //newWP.HoldTime.value = 2;
        //newWP.Event_Flag.value = 0;
        //newWP.Index.value = 1;
        //newWP.Type.value = 0;
        //newWP.WP_EventChannelValue.value=100;
        //newWP.AltitudeRate.value = 30;
        //newWP.Speed.value = 30;
        //newWP.CameraAngle.value = 0;
        //encoder.send_command(2,'w',newWP.getAsInt());
        
        /** (NAVI) SET 3D DATA INTERVAL**/
        //u8 interval = new u8("interval");
        //interval.value = 100; //PASSING 0 STOPS THE STREAM
        //encoder.send_command(2,'c',interval.getAsInt());
        
        /** (FLIGHT) Engine Test **/
        //int motor[] = new int[16];
        //motor[0] = 10;
        //motor[1] = 10;
        //motor[2] = 10;
        //motor[3] = 10;
        //motor[4] = 10;
        //motor[5] = 10;
        //encoder.send_command(1,'t',motor);
      
        /** (NAVI) ERROR TEXT REQUEST**/
        //encoder.send_command(2,'e');
        
        /** (COMMON) Version Request **/ //DATA OK?
        //encoder.send_command(0,'v');
        
        /** (COMMON) Request Display h **/
        //encoder.send_command(0,'h',1000);
        
        /** (COMMON) l REQUEST DIPLAY **/
        //u8 menuItem = new u8("menuItem");
        //menuItem.value = 1; //from 1 to 19!!
        //encoder.send_command(0,'l',menuItem.getAsInt());
        
        
        /** (NAVI) BL CTRL Status: BLDataStruct for every motor**/
        //u8 interval = new u8("interval");
        //interval.value = 3000; 
        //encoder.send_command(2,'k',interval.getAsInt());
	}

}
