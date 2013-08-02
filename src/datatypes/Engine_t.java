package datatypes;

import java.util.LinkedList;

public class Engine_t extends c_int {
	u8 Engines[] = new u8[16];
	
	public Engine_t(){
		allAttribs = new LinkedList<c_int>();
		
        for (int i = 0; i < Engines.length; i++) {
        	Engines[i] = new u8("engine_" + i);
            allAttribs.add(Engines[i]);
        }
	}
	
	public void setValue(int id, long value){
		allAttribs.get(id).setValue(value);
	}
}
