package datatypes;


public class s16 extends c_int{
	public s16() {
	        signed = true;
	        length = 16;
	        this.name = "";
	}
	
	public s16(String name) {
	    signed = true;
	    length = 16;
	    this.name = name;
	}
	
    /**
     * @param minValue The chosen value can not be below this minValue for safety 
     * @param maxValue	The chosen value can not be above this maxValue for safety
     * **/
	public s16(int minValue, int maxValue) {
	    signed = true;
	    length = 16;
	    this.maxValue = maxValue;
	    this.minValue = minValue;
	}
	
    /**
     * @param minValue The chosen value can not be below this minValue for safety 
     * @param maxValue	The chosen value can not be above this maxValue for safety
     * **/
	public s16(String name,int minValue, int maxValue) {
	    signed = true;
	    length = 16;
	    this.name = name;
	    this.maxValue = maxValue;
	    this.minValue = minValue;
	}
}
