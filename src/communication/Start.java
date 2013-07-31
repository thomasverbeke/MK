package communication;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import datatypes.u16;
import datatypes.u8;

public class Start {
	
	 public static void main(String[] args) throws IOException { 
	        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
	        System.out.println("MK CMDLINE APP");
	        System.out.println("Enter Command");
	        SerialReader reader = null;

	        while (true){
		        String s = br.readLine();
		        
		        try {
					reader = new SerialReader();
					
					u8 SatsInUse = new u8("SatsInUse"); 						// number of satellites used for position solution
					SatsInUse.setValue(7);
					SatsInUse.getAsInt();
					
					u16 GroundSpeed = new u16("GroundSpeed");
					GroundSpeed.setValue(10);	// speed over ground in cm/s (2D)
					GroundSpeed.getAsInt();
					
				} catch (Exception e) {
					System.out.print("Error in serialReader");
					e.printStackTrace();
				}
		        
		        if (s.equals("help")){
		        	System.out.println("Listing commands:");
		        	System.out.println("*listen* start listening for incoming serial frames");
		        	System.out.println("*send* send a frame to MK; ex to follow");
		        	System.out.println("*encode* encodes a frame; ex to follow ");
		        	System.out.println("*decode* decodes a frame; ex to follow ");
		        } else if (s.equals("listen")){
		//LISTEN
			        //list ports
		        	try {
						reader.Listen(null);
					} catch (Exception e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
		        	
		        } else if (s.equals("send")){
		//SEND 
		        //when sending there is also a possibility to enable	
		        	
		        } else if (s.equals("encode")){
		//ENCODE 
		        
		        	
		        } else if (s.equals("decode")){
		//DECODE        	
		        	
		        	int[] dataFrame = new int [10];
		   
		        	
		        	reader.decode_buffer(dataFrame);
		        		
		        } else if (s.equals("exit")){
		//EXIT
		        	System.out.println("Exiting");
		        	return;
		        } else {
		        	System.out.println("Command not recognised");
		        	System.out.println("input:"+s);
		        }
		        
		        try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} 
	        }   
	    }
}
