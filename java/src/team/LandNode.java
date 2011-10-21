package team;

public class LandNode extends Node{

    private int id;
    public LandNode(int pos, double x, double y, int id){
	state = new double[2];
	state[0] = x;
	state[1] = y;
	this.id = id;
    }
    
    public int getId(){
	return id;
    }

    public String toString(){
	return "Land Node @ " + state[0] + " " + state[1] + " ID: " + getId();
    }
  

}