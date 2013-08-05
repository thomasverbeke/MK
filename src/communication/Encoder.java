/** 
 * @author Thomas Verbeke
 * 
 * This class should be executed when a command needs to be send.
 * TODO Needs to changed so it can generate codes without sending them
 * this is needed for just making commands in a test environment.
 * **/

package communication;

import java.io.OutputStream;
import java.util.Arrays;

/** Notes
 * 
 * I have noticed sending frames without having a delay in the codes causes problems for the MK. (I ignores send commands... => Investigate)
 * 
 * **/

public class Encoder {

   public java.io.OutputStream writer;

   public Encoder(OutputStream writer) {
       this.writer = writer;
   }

   public void setWriter(OutputStream writer) {
       this.writer = writer;
   }

   /**
    *
    * @param modul : destination module
    * @param cmd : serial frame command
    * @param params : data (type according to command)
    *
    * @author  Marcus -LiGi- Bueschleb
    * http://github.com/ligi/DUBwise/blob/master/shared/src/org/ligi/ufo/MKCommunicator.java
    * java byte = 8 bit signed 2-complement
    */
   public synchronized void send_command_nocheck(byte modul, char cmd, int[] params) {

       //DataStorage.statusBar.uartTX.toggle();

       //writer = SerialReader.outputStream;
       if (writer != null) {
           byte[] send_buff = new byte[3 + (params.length / 3 + (params.length % 3 == 0 ? 0 : 1)) * 4]; // 5=1*start_char+1*addr+1*cmd+2*crc
           send_buff[0] = '#';
           send_buff[1] = (byte) (modul + 'a'); // FC address = 'a' + 1 -> ascii 'b' or decimal 98 for example
           send_buff[2] = (byte) cmd;
           
           

           for (int param_pos = 0; param_pos < (params.length / 3 + (params.length % 3 == 0 ? 0 : 1)); param_pos++) {
               int a = (param_pos * 3 < params.length) ? params[param_pos * 3] : 0;
               int b = ((param_pos * 3 + 1) < params.length) ? params[param_pos * 3 + 1] : 0;
               int c = ((param_pos * 3 + 2) < params.length) ? params[param_pos * 3 + 2] : 0;

               send_buff[3 + param_pos * 4] = (byte) ((a >> 2) + '=');
               send_buff[3 + param_pos * 4 + 1] = (byte) ('=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4)));
               send_buff[3 + param_pos * 4 + 2] = (byte) ('=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6)));
               send_buff[3 + param_pos * 4 + 3] = (byte) ('=' + (c & 0x3f));

           }
           try {
               int tmp_crc = 0;
               for (int tmp_i = 0; tmp_i < send_buff.length; tmp_i++) {
                   tmp_crc += (int) send_buff[tmp_i];
               }

  

               writer.write(send_buff, 0, send_buff.length);
               tmp_crc %= 4096;



               writer.write((char) (tmp_crc / 64 + '='));
               writer.write((char) (tmp_crc % 64 + '='));
               writer.write('\r');
               writer.flush();


               String out = "";
               for (int i : send_buff) {
                   out += (char) i;
               }

               out += (char) (tmp_crc / 64 + '=');
               out += (char) (tmp_crc % 64 + '=');
               out += '\r';
                
               System.out.println("DEBUG<encode(send_command_nocheck)> "+out);
               //TODO Test this
               Thread.sleep(100);
           } catch (Exception e) { // problem sending data to FC
        	   System.out.println("problem sending data");
           }
       }
   }
   
   

   /**
    *
    * @param modul
    * @param cmd
    * @param params
    *
    * @author  Marcus -LiGi- Bueschleb
    * http://github.com/ligi/DUBwise/blob/master/shared/src/org/ligi/ufo/MKCommunicator.java
    */
   public void send_command(int modul, char cmd, int[] params) {
       send_command_nocheck((byte) modul, cmd, params);
   }
   
   public void send_command(int modul, char cmd) {
       send_command(modul, cmd, new int[0]);
   }
  
   public void send_command(int modul,char cmd,int param)
   {
	int[] params=new int[1];
	params[0]=param;
	send_command(modul,cmd,params);
   }
   
   //#define REQUEST_NC_VERSION "#bv====Dl\r"


   /**
    * Verify the checksum in a received MK-Dataframe
    *
    * @param buffer byte array containing the received bytes
    * @return true if received checksum is valid, fals otherwise
    */
   public static boolean mkCRC(byte[] buffer) {
       int buf_ptr = 0, end;

       while (buffer[buf_ptr] != '#' && buf_ptr < buffer.length) {
           buf_ptr++;
       }

       end = buf_ptr;
       while (buffer[end] != '\r' && end < buffer.length) {
           end++;
       }
       return mkCRC(buffer, buf_ptr, end);
   }

   /**
    * Verify the checksum in a received MK-Dataframe
    *
    * @param buffer byte array containing the received bytes
    * @param buf_ptr index of the #
    * @param end index of the \r
    * @return true if received checksum is valid, fals otherwise
    */
   public static boolean mkCRC(byte[] buffer, int buf_ptr, int end) {
       int crc = 0;
       int crc1, crc2;

       for (int i = buf_ptr; i < end - 2; i++) {
           crc += buffer[i];
       }
       crc %= 4096;
       crc1 = '=' + crc / 64;
       crc2 = '=' + crc % 64;

       if ((crc1 == buffer[end - 2]) && (crc2 == buffer[end - 1])) {
           return true;
       }
       return false;
   }
   
    /** When re-directing the UART using the 'u' cmd frame; we need a way to switch back to navi-ctrl.
     * 	This happens by sending a so called "magic packet"
     * 	"0x1B,0x1B,0x55,0xAA,0x00" 
     * **/
    public synchronized void send_magic_packet() {

       //DataStorage.statusBar.uartTX.toggle();

       writer = SerialReader.outputStream;
       
       if (writer != null) {
           
           //magic packet to switch to navi
           byte[] magic={27,27,0x55,(byte)0xAA,0,(byte)'\r'};
           
           try {   
               writer.write(magic);
               writer.flush();
               
           } catch (Exception e) { // problem sending data to FC
               
           }
       }
   }
    
    
}
