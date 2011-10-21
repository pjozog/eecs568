package team;

public class OdNode extends Node{

    public OdNode(double x, double y, double t){
	state = new double[3];
	state[0] = x;
	state[1] = y;
	state[2] = t;
    }

}