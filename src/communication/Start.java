package communication;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class Start {
	 public static void main(String[] args) throws IOException { 
	        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
	        System.out.println("MK CMDLINE APP");
	        System.out.println("Enter Command");
	        
	        while (true){
		        String s = br.readLine();
		        
		        if (s.equals("help")){
		        	System.out.println("Listing commands:");
		        	System.out.println("*start* will startup the porgram with it's default port");
		        } else if (s.equals("start")){
	
			        //list ports
			        try {
						SerialReader reader = new SerialReader(null);
					} catch (Exception e) {
						System.out.print("Error in serialReader");
						e.printStackTrace();
					}
		        	
		        } else if (s.equals("exit")){
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
