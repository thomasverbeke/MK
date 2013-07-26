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
}
