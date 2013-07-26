package communication;

import java.io.InputStream;
import java.io.OutputStream;

public class CommunicationBase {
    // slave addresses http://svn.mikrokopter.de/filedetails.php?repname=NaviCtrl&path=%2Ftags%2FV0.20c%2Fmkprotocol.h
    public static final int ANY_ADDRESS = 0;
    public static final int FC_ADDRESS = 1;
    public static final int NC_ADDRESS = 2;
    public static final int MK3MAG_ADDRESS = 3;
    public static final int MKOSD_ADDRESS = 4;
    public static final int BL_ADDRESS = 5;
    
    InputStream inputStream;
    static OutputStream outputStream;

    public OutputStream getOutputStream() {
        return outputStream;
    }
    
}
