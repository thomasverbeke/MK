package communication;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import datatypes.u16;

public class SpeedOfCommTest {
	/** 
	 *  This class is written for the purpose of testing and documenting speed of frame transmission.
	 *  How long does it take for a command to reach the MK?
	 *  We can guess factors like distance, interference, type of command, MK clock speed will affect speed
	 *  The meaning of the class is also to test how the MK reacts under stress: heavy load of commands.
	 *  We will try to reach it's limits to see what they are and how the copter reacts under these types of situations.
	 *  
	 * **/
	

	//send a suite of frames
	//we can send them serially (always waiting for an ACK to come back first)
	//or we can send a bunch of commands in rapid successsion.
	
	//how are we going to achieve this?
	//start up a serial connection. Disable all auto send data? 
	//log data using a logger factory!!
	
	//which command?
	//=> serialtest: Ideal candidate; because we can identify the frame by the echoPattern
	
	//disable all other comm
	//send a simple serial;
	//wait a number of ms
	//resend another
	public Map<u16,Long> map=new ConcurrentHashMap<u16,Long>();
	private u16 ID;
	private long timeStamp;
	
	public SpeedOfCommTest(){
		
		SerialReader reader = new SerialReader();
		OutputStream stream;
		Writer fileWriter = null;
		SerialWriter writer = null;
		/** Open the ports & start listening for incoming frames**/
		try {
			stream = reader.Listen("/dev/cu.usbserial-A400782N");
			
			if (stream == null){
				//something went wrong
				//http://stackoverflow.com/questions/3715967/when-should-we-call-system-exit-in-java
				System.exit(0);
			} else {
				 writer = new SerialWriter(stream);
			}
		} catch (Exception e) {
			System.out.println("Something went wrong while starting the serialReader");
			e.printStackTrace();
		}
		
		/**  Disable all interval frames **/
		//disable DebugReqDataInterval
		//disable 3DDataInterval
		//disable OSDDataInterval
		//disable ReqDisplay
		//disable BLStatusInterval
		//disable SystemTimeInterval
		writer.sendCommand("test");

		try {
			fileWriter = new BufferedWriter(new OutputStreamWriter(
		          new FileOutputStream("report.txt"), "utf-8"));
			fileWriter.write("Speed Com Test");
		} catch (IOException ex){
			System.out.println("Something went wrong");
		} finally {
		   try {
			   fileWriter.close();
		   } catch (Exception ex) {}
		}	
	}
}
