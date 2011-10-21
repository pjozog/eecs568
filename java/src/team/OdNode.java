package team;

public class OdNode extends Node{

    public OdNode(int pos, double x, double y, double t){
	posInStateVector = pos;
	state = new double[3];
	state[0] = x;
	state[1] = y;
	state[2] = t;
    }
    public String toString(){
	return "Od Node @ " + state[0] + " " + state[1] + " " + state[2];
    }

}