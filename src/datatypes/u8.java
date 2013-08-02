package datatypes;

public class u8 extends c_int {
	 	public u8() {
	        signed = false;
	        length = 8;
	        this.name = "";
	    }

	    public u8(String name) {
	        signed = false;
	        length = 8;
	        this.name = name;
	    }
	    
	    /**
	     * @param minValue The chosen value can not be below this minValue for safety 
	     * @param maxValue	The chosen value can not be above this maxValue for safety
	     * **/
	    public u8(int minValue, int maxValue) {
	        signed = false;
	        length = 8;
	        this.minValue = minValue;
	        this.maxValue = maxValue;
	    }
	    
	    /**
	     * @param minValue The chosen value can not be below this minValue for safety 
	     * @param maxValue	The chosen value can not be above this maxValue for safety
	     * **/
	    public u8(String name, int minValue, int maxValue) {
	        signed = false;
	        length = 8;
	        this.name = name;
	        this.minValue = minValue;
	        this.maxValue = maxValue;
	    }
}
