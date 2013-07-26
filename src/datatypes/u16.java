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
}