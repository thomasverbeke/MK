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
}
