package datatypes;

import java.util.LinkedList;

/**
 * GPS Position Deviation Struct
 * adapted from: http://svn.mikrokopter.de/
 * navictrl - v0.28i - uart1.h
 */
public class GPS_PosDev_t extends c_int {

    public u16 Distance;        // distance to target in dm
    public s16 Bearing;         // course to target in deg

    public GPS_PosDev_t(String prefix) {
        Distance = new u16(prefix + " Distance");
        Bearing = new s16(prefix + " Bearing");
        allAttribs = new LinkedList<c_int>();
        allAttribs.add(Distance);
        allAttribs.add(Bearing);
    }

    @Override
    public int[] getAsInt() {
        return c_int.concatArray(Distance.getAsInt(), Bearing.getAsInt());
    }
}