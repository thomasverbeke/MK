package datatypes;

public class s8 extends c_int {
	public s8(){
		 signed = false;
	     length = 8;
	}
	
	public s8(String name) {
        signed = true;
        length = 8;
        this.name = name;
    }
	
	/**
     * @param minValue The chosen value can not be below this minValue for safety 
     * @param maxValue	The chosen value can not be above this maxValue for safety
     * **/
	public s8(int minValue, int maxValue) {
        signed = true;
        length = 8;
        this.minValue = minValue;
        this.maxValue = maxValue;
    }
	
	/**
     * @param minValue The chosen value can not be below this minValue for safety 
     * @param maxValue	The chosen value can not be above this maxValue for safety
     * **/
	public s8(String name, int minValue, int maxValue) {
        signed = true;
        length = 8;
        this.name = name;
        this.minValue = minValue;
        this.maxValue = maxValue;
    }
}
