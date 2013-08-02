package datatypes;

import java.io.ByteArrayOutputStream;
import java.io.ObjectOutputStream;
import java.util.LinkedList;

/** Do we really need long? Is char not enough?**/
public abstract class c_int {

    public boolean signed;
    public int length = 0;
    public long value = 0;
    public String name;
    protected Integer minValue; //protected means world has no access but subclass does
    protected Integer maxValue;
    
    //linked list is not synchronized!!
    public LinkedList<c_int> allAttribs = null;
    
    public LinkedList<c_int> getList() {
        LinkedList<c_int> poss = new LinkedList<c_int>();
        if (allAttribs == null) {
            poss.add(this);
        } else {
            for (c_int c : allAttribs) {
                poss.addAll(c.getList());
            }
        }
        return poss;
    }

    //get number of elements inside the linkedlist; the list can contain sublist..these are counted also
    public int getLength() {
        if (allAttribs == null) {
            return 0;
        } else {
            int len = 0;
            // enhanced for loop; each element of the allAttribs list is cast to c_int
            for (c_int c : allAttribs) {
                len += c.getLength();
            }
            return len;
        }
    }

    /** loadFromInt() function 
     * 	@param decodedDataFrame : int[] containing the whole decoded data frame
     *  @param dataPointer : indicates starting position of data in the decodedDataFrame
     *  
	 * --Frame Structure-- (http://www.mikrokopter.de/ucwiki/en/SerialProtocol)
	 * 	0 - Start-Byte: 			'#'
	 * 	1 - Address Byte: 			'a'+ Addr
	 * 	2 - ID-Byte:				'V','D' etc'
	 * 	3 - n Data-Bytes coded: 	"modified-base64"
	 * 	? - CRC-Byte1:				variable
	 * 	? - CRC-Byte2:				variable
	 * 	? - Stop-Byte:				'\r'
	**/
    //dataPointer is always 3
    public void loadFromInt(int[] decodedDataFrame, int dataPointer) {
        if (allAttribs != null && allAttribs.size() > 0) {
            int offset = 0;
            for (c_int c : allAttribs) {
                c.loadFromInt(decodedDataFrame, dataPointer + offset);
                offset += c.getLength() / 8;
            }
      
        } else {
        	//long data type is a 64-bit signed
            long v = 0;
            if (dataPointer + (getLength() / 8) <= decodedDataFrame.length) {//???? OVERBODIG?
                for (int i = (length / 8) - 1; i >= 0; i--) {
                	//<< signed left shift; won't shift the sign!
                    v = v << 8; //shift 8 bit to the left
                    //v <<= 8;
                    //| Bitwise  OR
                    //int is only 32-bit signed
                    v = v | decodedDataFrame[i + dataPointer];
                }
               
                if (signed) {
                    long signmask = 1 << (getLength() - 1); //Remember 1 is stored as it's 2-complement (bitwise complement +1; we remove 1)
                    long signbit = ((v & signmask) != 0) ? 1 : 0;
                    v &= ~signmask; //Bitwise AND and bitwise complement
                    if (signbit == 1) {
                        v = v + getMin();
                    }
//                  System.out.println(v);
                }
                setValue(v);
            } else {
                //System.out.println("\tBuffer to short?");
            }
        }
    }
    
    public int getMin() {
        if (!signed) {
            return 0;
        } else {
            if (minValue != null) {
                return minValue.intValue();
            }
            return (int) -(Math.pow(2, length - 1));
        }
    }
    
    public void setValue(long value) {
        this.value = value;
    }


   /** getAsInt() function
    * 
    *  will convert all c_int datatypes (u8,u16,u32,s8,s16,s32) to an int[] where int[0] contains the first 8 bits, int[1] the next 8 bits,...
    *  
    * **/
    public int[] getAsInt() {
        if (allAttribs == null) {
        	//convert data
            int[] ret = new int[length / 8];
            for (int i = 0; i < ret.length; i++) {
            	//if datatypes has more than 8 bits; we will move the bits around (>> shift 8 bits to the right)
            	//By ANDing with 0xff we retain/select only the lowest 8 bits
            	//00000000 00000000 00000000 11111111
            	//this way we get to contain all information inside ret[]
            	//more info: http://stackoverflow.com/questions/3845834/understanding-java-bytes
                ret[ret.length - 1 - i] = (char) (0xff & (value >> ((ret.length - i - 1) * 8)));
            }
            return ret;
        } else {
        	//loop over all attributes
            int[] ret = new int[0];
            for (c_int o : allAttribs) {
            	//we concat all the converted data
                ret = concatArray(ret, o.getAsInt());
            }
            return ret;
        }
    }

    /**
     * Concat two integer arrays
     * @param a one array
     * @param b another array
     * @return ab
     */
    public static int[] concatArray(int[] a, int[] b) {
        if (a.length == 0) {
            return b;
        }
        if (b.length == 0) {
            return a;
        }
        int[] ret = new int[a.length + b.length];
        for (int i = 0; i < a.length; i++) {
            ret[i] = a[i];
        }
        for (int i = a.length; i < a.length + b.length; i++) {
            ret[i] = b[i - a.length];
        }
        return ret;
    }

    
    public void printOut() {
        if (allAttribs != null) {
            for (c_int c : allAttribs) {
                c.printOut();
            }
        } else {
            System.out.println(name + "\t" + value);
        }
    }

}

