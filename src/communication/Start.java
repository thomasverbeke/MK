package communication;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;

import datatypes.u16;
import datatypes.u8;

public class Start {
	
	 public static void main(String[] args) throws IOException { 
	        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
	        System.out.println("MK CMDLINE APP");
	        System.out.println("Enter Command");
	        SerialReader reader = null;	 
	        OutputStream outStream;

	        while (true){
		        String mainCmd = br.readLine();

		        try {
					reader = new SerialReader();
					
				} catch (Exception e) {
					System.out.print("Error in serialReader");
					e.printStackTrace();
				}
		        
		        if (mainCmd.equals("help")){
		        	System.out.println("Listing commands:");
		        	System.out.println("*listen* start listening for incoming serial frames");
		        	System.out.println("*send* send a frame to MK; ex to follow");
		        	System.out.println("*encode* encodes a frame; ex to follow ");
		        	System.out.println("*decode* decodes a frame; ex to follow ");
		        } else if (mainCmd.equals("listen")){
		//LISTEN
		        	System.out.println("Type in a port to listen to (or press enter for default)");
		        	String listen = br.readLine();	        	
		        	try {
		        		//open in new thread?
		        		if (listen.equals("")){
		        			outStream = reader.Listen(null);
		        		} else {
		        			outStream = reader.Listen(listen);
		        		}
		        		
		        		//stay in loop until 
					} catch (Exception e1) {
						System.out.println("Exception while trying to listen");
						e1.printStackTrace();
					}
	
		        } else if (mainCmd.equals("send")){
		//SEND 		
		        	System.out.println("Before sending we need to open a serial port");
		        	System.out.println("Type in a port to listen to (or press enter for default)");
		        	String listen = br.readLine();	        	
		        	try {
		        		//open in new thread?
		        		if (listen.equals("")){
		        			outStream = reader.Listen(null);
		        		} else {
		        			outStream = reader.Listen(listen);
		        		}
		        		
		        		System.out.println("Enter the frametype you wish to test");
		        		System.out.println("**FrameTypes**");
		        		System.out.println("*version*");
		        		System.out.println("*DebugReqName*");
		        		System.out.println("*DebugReqDataInterval*");
		        		System.out.println("*BLStatusInterval*");
		        		System.out.println("*redirectUART*");
		        		System.out.println("*errorText*");
		        		System.out.println("*sendTarget*");
		        		System.out.println("*sendWP*");
		        		System.out.println("*serialTest*");
		        		System.out.println("*systemTimeInterval*");
		        		System.out.println("*3DDataInterval*");
		        		System.out.println("*OSDDataInterval*");
		        		System.out.println("*ReqDisplay*");
		        		System.out.println("*EngineTest*");
		
		        		String send = br.readLine();	
		        		SerialWriter writer = new SerialWriter(outStream);
		        		writer.sendTestCommand(send);
						
					} catch (Exception e1) {
						System.out.println("Exception while trying to listen");
						e1.printStackTrace();
					}
		        	
		        } else if (mainCmd.equals("decode")){
		//DECODE        	
		        	System.out.println("Enter the frame you wish to decode");
		        	Encoder encoder = new Encoder(null);
		        	String decode = br.readLine();	 
		        	char[] decodeArray = decode.toCharArray();
		        	
		        	for (int i=0; i<decode.length(); i++) {
                    	reader.UART_vect(decodeArray[i]);
                    }
		        		
		        } else if (mainCmd.equals("exit")){
		//EXIT
		        	System.out.println("Exiting");
		        	return;
		        } else {
		        	System.out.println("Command not recognised");
		        	System.out.println("input:"+mainCmd);
		        }
		        
		        try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					System.out.println("Exception while trying to sleep");
					e.printStackTrace();
				} 
	        }   
	    }
}
