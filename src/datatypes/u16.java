package datatypes;


public class u16 extends c_int {
	public u16() {
        signed = false;
        length = 16;
        this.name = "";
    }
	
    public u16(String name) {
        signed = false;
        length = 16;
        this.name = name;
    }
    
    /**
     * @param minValue The chosen value can not be below this minValue for safety 
     * @param maxValue	The chosen value can not be above this maxValue for safety
     * **/
    public u16 (int minValue, int maxValue) {
        signed = false;
        length = 16;
        this.minValue = minValue;
        this.maxValue = maxValue;
    }
    
    /**
     * @param minValue The chosen value can not be below this minValue for safety 
     * @param maxValue	The chosen value can not be above this maxValue for safety
     * **/
    public u16(String name, int minValue, int maxValue) {
        signed = false;
        length = 16;
        this.name = name;
        this.minValue = minValue;
        this.maxValue = maxValue;
    }
    
}