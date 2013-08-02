package datatypes;

public class s32 extends c_int {
	public s32(){
		signed=true;
		length=32;
	}
	
	public s32(String name) {
        signed = true;
        length = 32;
        this.name = name;
    }
	
    /**
     * @param minValue The chosen value can not be below this minValue for safety 
     * @param maxValue	The chosen value can not be above this maxValue for safety
     * **/
	public s32(int minValue, int maxValue) {
        signed = true;
        length = 32;
        this.minValue = minValue;
        this.maxValue = maxValue;
    }
	
    /**
     * @param minValue The chosen value can not be below this minValue for safety 
     * @param maxValue	The chosen value can not be above this maxValue for safety
     * **/
	public s32(String name, int minValue, int maxValue) {
        signed = true;
        length = 32;
        this.name = name;
        this.minValue = minValue;
        this.maxValue = maxValue;
    }
}
