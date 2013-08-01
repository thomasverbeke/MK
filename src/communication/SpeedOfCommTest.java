package communication;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;

public class SpeedOfCommTest {
	/** 
	 *  This class is written for the purpose of testing and documenting speed of frame transmission.
	 *  How long does it take for a command to reach the MK?
	 *  We can guess factors like distance, interference, type of command, MK clock speed will affect speed
	 *  The meaning of the class is also to test how the MK reacts under stress: heavy load of commands.
	 *  We will try to reach it's limits to see what they are and how the copter reacts under these types of situations.
	 *  
	 * **/
	public SpeedOfCommTest(){
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

	
		Writer writer = null;

		try {
		    writer = new BufferedWriter(new OutputStreamWriter(
		          new FileOutputStream("report.txt"), "utf-8"));
	  		      writer.write("Speed Com Test");
		} catch (IOException ex){
			System.out.println("Something went wrong");
		} finally {
		   try {
			   writer.close();
		   } catch (Exception ex) {}
		}
		
	}
}
