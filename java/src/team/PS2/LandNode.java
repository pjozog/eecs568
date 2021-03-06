/** EECS 568 Homework 2, SLAM
Steve Chaves
Schuyler Cohen
Patrick O'Keefe
Paul Ozog
**/


package team.PS2;

public class LandNode extends Node{

    private int id;
    public LandNode(int pos, int absPos, double x, double y, int id){
	posInStateVector = pos;
	absIndexInStateVector = absPos;
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
