package test;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;

import communication.Encoder;
import communication.SerialReader;
import datatypes.VersionInfo_t;
import datatypes.u16;

public class HandleFrameTest {
	
    public static final byte ANY_ADDRESS = 0;
    public static final byte FC_ADDRESS = 1;
    public static final byte NC_ADDRESS = 2;
    public static final byte MK3MAG_ADDRESS = 3;
    public static final byte MKOSD_ADDRESS = 4;
    public static final byte BL_ADDRESS = 5;
    
    public static void main(String[] args) throws IOException { 
    	/** Instead of writing to a serial port we will write to a self-expanding buffer **/
		ByteArrayOutputStream buffer_out = new ByteArrayOutputStream();
		Encoder encoder = new Encoder(buffer_out);
		SerialReader reader = new SerialReader();
		
		/** Test 1: NC - Z (=SerialTest ACK) **/
		u16 echo = new u16("echo");
		echo.value=169;
		encoder.send_command(NC_ADDRESS,'Z',echo.getAsInt());	
		
		/** Test 2: FC - T (=Feedback from 'BL Ctrl Status)**/
		encoder.send_command(FC_ADDRESS,'T',0);
		
		/** Test 3: ANY - V (=Version Info)**/
		//just send it as empty
		VersionInfo_t versionStruct = new VersionInfo_t("versioninfo");
		encoder.send_command(ANY_ADDRESS,'V',versionStruct.getAsInt());
		
		byte[] data = buffer_out.toByteArray();
		ByteArrayInputStream buffer_in = new ByteArrayInputStream(data);
		try {
			 while (buffer_in.available() > 0) {
			
				byte[] readBuffer = new byte[220];
				int numBytes = buffer_in.read(readBuffer);
			
				for (int i=0; i<numBytes; i++) {
		        	reader.UART_vect((char) readBuffer[i]);
		        }
			
			}
		} catch (IOException ex) {
			System.out.println("Something when wrong while reading buffer");
        }
    }
}
