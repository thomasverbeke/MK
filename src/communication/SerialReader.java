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
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.UnsupportedEncodingException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;

import gnu.io.*;

import datatypes.*;

public class SerialReader extends CommunicationBase implements Runnable,SerialPortEventListener {
	
	/** The main purpose of the class is to setup the serial connection with the MK.
	 *  While also allowing for some functions to be used externally. 
	 *  In some debug/test situations it might come in handy to manually assemble packets,...  **/
		
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
	private boolean speedTest = false;
	 
	/** getPorts method
	 * Mapping the (serial) ports to a HashMap
	 * source: http://rxtx.qbang.org/wiki/index.php/Discovering_comm_ports
	 * **/
	public static HashMap<String, CommPortIdentifier> getPorts() {
        if (portMap == null) {
            portMap = new HashMap<String, CommPortIdentifier>();
            Enumeration portList = CommPortIdentifier.getPortIdentifiers();
            System.out.println("Printing ports:");
           
            while (portList.hasMoreElements()) {
                portId = (CommPortIdentifier) portList.nextElement();   
                //we are only interested in the serial ports
                if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
                    portMap.put(portId.getName(), portId);
                    System.out.println("-portName: " + portId.getName() + "   -portID: " + portId);
                }
            }
        }
        return portMap;
    }
	
	public Map<u16,Long> map;
	public Writer fileWriter = null;
	/** We need to transfer the shared map**/
	public void setSpeedTest (boolean bool, Map<u16,Long> map){
		try {
			fileWriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream("report_result.txt"), "utf-8"));
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			System.out.println("Failed to open new file to write speedTest results to");
			e.printStackTrace();
		}
		
		this.map = map;
		speedTest = bool;
	}

	public SerialReader(){
		
	}
        
	/** SerialReader Method 
	 * @param port : Port to connect with; if port is null a default port is choosen depending on the operating system
	 * 
	 *  List ports
	 *  Connect trough default port with MK hardware.
	 * **/
	public OutputStream Listen(String port) throws Exception {
			getPorts();
			isUSB =false; //TODO change isUSB implementation?
			
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
	            return null;
	        }

	        //no port set -> take default port
	        if (port != null) {
	            defaultPort = port;
	            
	        } else {
	        	 System.out.println("No port set: using default port :" + defaultPort);
	        }


	        // parse ports and if the default port is found, initialized the reader
	       
	        if (!portMap.keySet().contains(defaultPort)) {
	            System.out.println("port " + defaultPort + " not found.");
	            System.out.println("Could not connect with default or chosen port");
	            return null;
	           // System.exit(0);
	      
	        } else {  	
	        	 /** see config.h  on http://svn.mikrokopter.de/ for UART settings **/
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
	             
	             readThread = new Thread(this);
	             readThread.start(); 
	             
	             return serialPort.getOutputStream();
	        }     
	}

	public void run() {
		while (true) {
            try {          
            	Thread.sleep(3000); 
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
	 
	/** serialEvent method
	 * 	@param event: handles port events
	 *  Important to note we use the RXTX (http://rxtx.qbang.org/wiki/index.php/Download)
	 *  For specific platforms other jars & dll might be needed >> check docs 
	 *  The implementation does however suffer from a bug which also caused my system (OSX 10.8.2) to freeze when using debug mode 
	 *  http://serialio.com/support/jspCommAPI.php might be a better alternative: INVESTIGATE if time?? 
	 *  http://stackoverflow.com/questions/12317576/stable-alternative-to-rxtx
	 * 
	 * **/
	public void serialEvent(SerialPortEvent event) {
		
	        switch (event.getEventType()) {
	            //http://docs.oracle.com/cd/E17802_01/products/products/javacomm/reference/api/javax/comm/SerialPortEvent.html
	            //we are not using javax.comm here but the page provides information on the general spec
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
	                        /** MAGIC PACKET
	                         *  http://forum.mikrokopter.de/topic-14090.html
	                         *  Magic packet to switch to navi-ctrl
	                         *  check in FIRMWARE
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
	
	
	/** UART_vect method
	 * @param SerialCharacter
	 *  Recieves serialcharacters from SerialPort to built up a MK serial frame
	 *  Once the frame has been assembled it is decoded in a new thread
	 *  Check http://www.mikrokopter.de/ucwiki/en/SerialProtocol for information about serial protocol
	 * **/
	
	public void UART_vect(char SerialCharacter) {

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

            		Thread thread = new Thread(new Runnable(){

					@Override
					public void run() {
						// TODO Auto-generated method stub
						handleFrame(RxdBuffer_work);
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
     * Decode64 method
     * Frame is encoded based on a special version of base64 encoding system. This code was based on work from:
     * @author Claas Anders "CaScAdE" Rathje
     * @author Marcus -LiGi- Bueschleb
     * which in turn looked at the decoding from the open source firmware & translated it to java
     * http://svn.mikrokopter.de/
     * v0.28i mkprotocol.c
     * 
     * **/
	public int[] Decode64(int[] RxdBuffer) {
       
		/** --Frame Structure-- (http://www.mikrokopter.de/ucwiki/en/SerialProtocol)
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
    //TODO remove name_counter
    public static int name_counter = 0;
    
	
    /** handleFrame method
     * @param dataFrame encoded dataFrame
     * 
     * main workhorse of the system: differentiate commands and act accordingly
     * **/
	public void handleFrame(int[] dataFrame) {
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
			System.out.println("NewDataRecieved (inside handleFrame): " + NewDataRecieved );
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
		
		int[] decodedDataFrame = Decode64(dataFrame); //DECODE THE FRAME; 
		
		 /**  
         * REMARK!
         * 
         * FC address = 'a' + 1 -> ascii 'b' or decimal 98 
         * NC address = 'a' + 2 -> ascii 'c' or decimal 99
         * 
         **/
		
		//TODO Add handler for SystemTime
		
		decodedDataFrame[1] = decodedDataFrame[1] - 'a';  //removing 'a'
		
		switch (decodedDataFrame[1]){
			case NC_ADDRESS:		//frame from Navi Ctrl
				switch (decodedDataFrame[2]){
					//NAVI-CTRL
		            case 'Z':   // Serial link test
		            	u16 echoPattern = new u16("echoPattern");
		            	echoPattern.loadFromInt(decodedDataFrame, dataPointer);	 
		            	if (speedTest){
		            		//read the map
		            		long estimated = map.get(echoPattern) - System.nanoTime();
		            		try {
		            			//write to log
		            			if (map.get(echoPattern) != null){
		            				fileWriter.write("ID: "+echoPattern.value+" - send on: "+map.get(echoPattern)+" time difference: "+estimated);
		            				map.remove(echoPattern);
		            			} else {
		            				System.out.println("Key not found in map; clearly something went wrong!!");
		            			}
			
							} catch (IOException e) {
								System.out.println("Failed to write speedTest result to file");
								e.printStackTrace();
							}
		            	} else {
			                System.out.println("<nZ>Serial Link Test");	                                             
			                System.out.println("u16 (echoPattern): " + echoPattern.value);
		            	}
		                break;    
		                            
		            case 'E':   // Feedback from Error Text Request; Error Str from Navi
		                System.out.println("<nE> Returns: Error Str from NaviCtrl");
		                //TODO Test this
		                /** Data is probably just an int[] of which every character has to be cast individually to a char
		                 * 	Perhaps it is advised to make add a function. 
		                 * **/
		                //char errorMessage= (char) decodedDataFrame[dataPointer];
		                String errorMessage = convertMessage(decodedDataFrame,dataPointer);    
		                System.out.println(errorMessage);		                             	                
		                break;
                  
		            case 'W':   // Feedback from 'send WP' command
		                System.out.println("<nW> Feedback from 'send WP' command - # Waypoint Index");
		                //System.out.println("<W> Returns: # Waypoints in memory");
		                //http://forum.mikrokopter.de/topic-post441164.html#post441164
		                //Returns the index of the WP that was added => see http://svn.mikrokopter.de/ : waypoints.c PointList_SetAt method
		                //Clearly this is a mistake in the documentation
		                
		                WPIndex = new u8("WPIndex");
		                WPIndex.loadFromInt(decodedDataFrame, dataPointer);
		                System.out.println("WPIndex (u8): " + WPIndex.value); 
		           
		                break;
		                
		            case 'X':   // Feedback from 'request WP' command
		                System.out.println("<nX> Returns: # Waypoints in memory; WP index; WP struct");	                
		                
		                /** -- Data Structure--
		                 * 
		                 * byte 1: u8 number of WP
		                 * byte 2: u8 WP index
		                 * from byte 3: Waypoint struct
		                */
		          	               
		                numberOfWP = new u8("numberOfWP");
		                numberOfWP.loadFromInt(decodedDataFrame, dataPointer);
		                System.out.println("NymberOfWP (u8): " + numberOfWP.value);
		                
		                WPIndex = new u8("WPIndex");
		                WPIndex.loadFromInt(decodedDataFrame, dataPointer+1);
		                System.out.println("WPIndex (u8): " + WPIndex.value);
		                
		             
		                Waypoint_t WP= new Waypoint_t("WP");
		                WP.loadFromInt(decodedDataFrame, dataPointer+2);
		                System.out.println("Latitude: "+WP.Position.Latitude.value);
		                System.out.println("Longitude: "+WP.Position.Longitude.value);
		                System.out.println("Altitude: "+WP.Position.Altitude.value);
		                System.out.println("Latitude: "+WP.Position.Latitude.value);
		                System.out.println("Speed: "+WP.Speed.value);
		                System.out.println("Altituderate: "+WP.AltitudeRate.value);
		                System.out.println("ToleranceRadius: "+WP.ToleranceRadius.value);
		                
		                break;    
	
		            case 'O':   // Feedback from 'request OSD Values'
		            	/** We will only print limited amount of data **/
		                System.out.println("<nO> Recieving OSD Data");
		                
		                NaviData_t navi = new NaviData_t();
		                navi.loadFromInt(decodedDataFrame,dataPointer); 
		             
		                System.out.println("Version: "+navi.Version.value);
		                System.out.println("CurrentPosition: (lat)"+navi.CurrentPosition.Latitude.value+" (lng) "+navi.CurrentPosition.Longitude.value+" (alt) "+navi.CurrentPosition.Altitude.value+" (status)" +navi.CurrentPosition.Status.value);
		                System.out.println("TargetPosition: (lat)"+navi.TargetPosition.Latitude.value+" (lng) "+navi.TargetPosition.Longitude.value+" (alt) "+navi.TargetPosition.Altitude.value+" (status)" +navi.TargetPosition.Status.value);
		                System.out.println("HomePosition: (lat)"+navi.HomePosition.Latitude.value+" (lng) "+navi.HomePosition.Longitude.value+" (alt) "+navi.HomePosition.Altitude.value+" (status)" +navi.HomePosition.Status.value);
		                System.out.println("SatsInUse: "+navi.SatsInUse.value);
		                System.out.println("Altimeter: "+navi.Altimeter.value);
		                System.out.println("Heading: "+navi.Heading.value);
		                System.out.println("Errorcode: "+navi.Errorcode.value);
		                   
		                break;
		                
		            case 'C':   // Feedback from 'set 3D data interval'
		                System.out.println("<nC> Recieving 3D Data");
		                /** We will only print limited amount of data **/
       
		                Data3D_t data3D = new Data3D_t();
		                data3D.loadFromInt(decodedDataFrame,dataPointer);

		                System.out.println("AngleNick: "+data3D.AngleNick.value);// AngleNick in 0.1 deg
		                System.out.println("AngleRoll: "+data3D.AngleRoll.value);// AngleRoll in 0.1 deg
		                System.out.println("Heading: "+data3D.Heading.value); // Heading in 0.1 deg
	
		                break; 
		            
		            case 'J':   // Feedback from 'Set/Get NC-Parameter' (only set when when there is a value
		                System.out.println("<nJ> Returns: ParameterId,Value");
		                
		                /** -- Data Structure--
		                 * 
		                 * byte 1: u8 parameterId
		                 * from byte 2: s16 value
		                */
		          	               
		                parameterId = new u8("parameterId");
		                parameterId.loadFromInt(decodedDataFrame, dataPointer);
		                System.out.println("Parameter Id (u8): " + parameterId.value);
		                
		                parameterValue = new s16("WPIndex");
		                parameterValue.loadFromInt(decodedDataFrame, dataPointer+1);
		                System.out.println("Parameter Value (s16): " + parameterValue.value);
                
		                break;   
		    
		            case 'K':   // Feedback from 'BL Ctrl Status'
		            	//TODO Handle different motors
		                BLData_t motorData = new BLData_t();
		                motorData.loadFromInt(decodedDataFrame, dataPointer);
		             
		                
		                System.out.println("<nK> Recieving BL Ctrl Data (DC Motor "+motorData.Index.value+" Data)");
		                int motorID = (int) motorData.Index.value;
		                motorList[motorID] = motorData;
		                           
		               break; 
		               
		            default: 
			        	   System.out.println("Unsupported command recieved"); 
			        	   System.out.println("CMD Id: "+decodedDataFrame[2]);
			        	   break;
					}
				break;
			case FC_ADDRESS:		//frame from Flight Ctrl
				switch (decodedDataFrame[2]){
					case 'T':   // Feedback from 'BL Ctrl Status'
						//!!Empty ack frame
		                System.out.println("<fT>Motor Test (empty frame)");
		                
		                break; 
		                
		            case 'P':   // Feedback from 'BL Ctrl Status'
		            	//TODO Test
		                System.out.println("<fP>Read PPM Channels");
		                //TODO There should be a constant somewhere containing the number of motors
		                int numOfMotors=8;
		                s16[] PPM_Array = new s16[numOfMotors];
		                String PPM_out = "";
		                for (int i=0; i<numOfMotors; i++){
		                	s16 PPM_Data = new s16();
		                	PPM_Data.loadFromInt(decodedDataFrame, dataPointer+i);
		                	PPM_Array[i] = PPM_Data;
		                	PPM_out += i + ": "+ PPM_Data.value + " ";
		                }
		                System.out.println(PPM_out);		                
		                break; 
           
		           case 'N':   // Feedback from 'Mixer Request'
		        	   	//TODO Test
		                System.out.println("<fN>Mixer request");
		                //revision
		                u8 MixerRevision = new u8();
		                MixerRevision.loadFromInt(decodedDataFrame, dataPointer);
	                	System.out.println("Revision: "+ MixerRevision.value);
		                
		                //name
		                u8[] Name_Array = new u8[12];
	                	String Mixer_out="";
		                for (int i=0; i<12; i++){
		                	u8 Name_Data = new u8();
		                	Name_Data.loadFromInt(decodedDataFrame, dataPointer+i);
		                	Name_Array[i] = Name_Data;
		                	Mixer_out += Name_Data.value;
		                }
		                System.out.println("Name: "+Mixer_out);
		                
		                //table
		                u8 MixerTable = new u8();
		                //TODO Add mixer table [16][4]      
		                break; 
		                
		           default: 
		        	   System.out.println("<f> Unsupported command recieved"); 
		        	   System.out.println("CMD Id: "+decodedDataFrame[2]);
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
			            	System.out.println("Label Index: "+analogLabelIndex.value);
			            			            	
			            	String analogLabelText = convertMessage(decodedDataFrame, dataPointer+1);
			            	System.out.println("Analog Label: " + analogLabelText);
			            	
			            	int convIndex = (int) analogLabelIndex.value;
			                names[convIndex]=analogLabelText;
			            
			                break;     
			                
				    case 'B':   // Feedback from 'External Control'
			                System.out.println("<B> Frame: unsigned char,echo of externCtrlStruct.Frame");
			                
			                /** 
			                 *  unsigned char Digital[2];
			                 *  unsigned char RemoteTasten;
			                 *  signed char   Nick;
			                 *  signed char   Roll;
			                 *  signed char   Gier;
			                 *  unsigned char Gas;
			                 *  signed char   Hight;
			                 *  unsigned char free;
			                 *  unsigned char Frame; => can be used as an ACK ID; this one is send back
			                 *  unsigned char Config; 
			                 *  
			                 * **/
			              
			                //TODO Test this conversion; maybe have a system remember the value and check against it
			                char Ext_frame = (char) decodedDataFrame[dataPointer];
			                System.out.println("confirmation: " +Ext_frame);
			                
				    		break;
			           
				    case 'H':   // Feedback from 'Request Display/LCD Data'	  1               
			                System.out.println("<H> Recieving Display Data");
			               
			                /** -- Data Structure--
			                 * 
			                 * byte 1-end: char[80] DisplayText
			                */
			                
			            
			                String displayMessage_h = convertMessage(decodedDataFrame, dataPointer);
			                System.out.println("Display Message: "+displayMessage_h);
			                
			                break;
			                
				    case 'L':   // Feedback from 'Request Display/LCD Data' 2
			                System.out.println("<L> Frame: MenuItem,MaxMenuItem,DisplayText");
		   
			                /** -- Data Structure--
			                 * 
			                 * byte 1: u8 MenuItem
			                 * byte 2: u8 MaxMenuItem   => WHAT IS THIS??
			                 * byte 3: char[80] Display Text
			                */
			                
			                u8 menuItem = new u8("MenuItem");
			                menuItem.loadFromInt(decodedDataFrame, dataPointer);
			                System.out.println("MenuItem: " +menuItem.value);
			                
			                u8 maxMenuItem = new u8("MaxMenuItem");
			                maxMenuItem.loadFromInt(decodedDataFrame, dataPointer+1);
			                System.out.println("maxMenuItem: " +maxMenuItem.value);
			                
			                String displayMessage_l = convertMessage(decodedDataFrame, dataPointer+2);
			                System.out.println("Display Message: "+displayMessage_l);
			                
			                break;
	
				    case 'V':   // Feedback from 'Version Info'
			                System.out.println("<V> Frame: VersionStruct");
			                
			                VersionInfo_t versionStruct = new VersionInfo_t("versioninfo");
			                versionStruct.loadFromInt(decodedDataFrame, dataPointer);
		   
			                System.out.println("SWMajor :"+ versionStruct.SWMajor.value);
			                System.out.println("SWMinor :"+ versionStruct.SWMinor.value);
			                System.out.println("ProtoMajor :"+ versionStruct.ProtoMajor.value);
			                System.out.println("ProtoMinor :"+ versionStruct.ProtoMinor.value);  
			                System.out.println("SWPatch :"+ versionStruct.SWPatch.value);
			                System.out.println("HardwareError :"+ versionStruct.HardwareError.toString()); //TODO TEST
			               
			                //TODO Document hardwareError codes http://www.mikrokopter.de/ucwiki/en/SerialCommands/VersionStruct

			                break;
			                
				    case 'D':   // Feedback from 'request debug Data'
			               	System.out.println("<D> Frame: Debug Request");
			               	//TODO check strange param ADRESS
			              
			                /** -- Data Structure--
			                 * 
			                 * unsigned char Status [2]
			                 * signed int Analog[32]
			                */
			               	
			               	String debug_status = "";
			               	debug_status += (char) decodedDataFrame[dataPointer];
			               	debug_status += (char) decodedDataFrame[dataPointer+1];
			               	
			               	int[] analog = new int[32];
			               	
			                //skip status and parse unsigned int
			                for (int i=2;i<34;i++){                 
			                	analog[i] = (int) decodedDataFrame[dataPointer+i];
			                }
			                
			                //TODO Better handle this
			                
			                System.out.println("Debug data recieved");
			                
			                break; 
			                
				    case 'G':   // Feedback from 'Get External control'	
			                System.out.println("<G> Returns: ExternControl Struct");
			                
			                //So what is the difference between this command and ExternCtrl? Since this is basically and empty frame (ack with externctrl struct)
			                //we can assume this enables the extern control?
			                //TODO Test
			                /** 
			                 *  unsigned char Digital[2];
			                 *  unsigned char RemoteTasten;
			                 *  signed char   Nick;
			                 *  signed char   Roll;
			                 *  signed char   Gier;
			                 *  unsigned char Gas;
			                 *  signed char   Hight;
			                 *  unsigned char free;
			                 *  unsigned char Frame; => can be used as an ACK ID; this one is send back
			                 *  unsigned char Config; 
			                 *  
			                 * **/
			              
			                //TODO Test this conversion; maybe have a system remember the value and check against it
			                //TODO Make a datatype externctrl (not a priority right now)
			                
				    		break;
				}
				break;
		}
		
	
	
        dataPointer = 0;
        numBytesDecoded = 0;
       
		
	}

	/** Convert the char[] type to a string for Error Message String frame type**/
	//TODO Add length parameter? Is this needed?
	public String convertMessage (int[] decodedDataFrame, int dataPointer){
    	String message = "";
    	//start at position 3!
    	for (int i=dataPointer; i<decodedDataFrame.length; i++){
            message += (char) decodedDataFrame[i];
        }
		return message;
    }

	public void initwritetoport() {
        // initwritetoport() assumes that the port has already been opened and

        try {
            // get the outputstream
            outputStream = serialPort.getOutputStream();
        } catch (IOException e) {
        	System.out.println("initwritetoport failed while trying to get outputstream");
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
