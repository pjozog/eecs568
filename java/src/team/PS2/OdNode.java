/** EECS 568 Homework 2, SLAM
Steve Chaves
Schuyler Cohen
Patrick O'Keefe
Paul Ozog
**/


package team.PS2;

public class OdNode extends Node{

    /*pos = node number, abs is overall index of the first index in the state*/
    public OdNode(int pos, int absPos, double x, double y, double t){
	posInStateVector = pos;
	absIndexInStateVector = absPos;
	state = new double[3];
	state[0] = x;
	state[1] = y;
	state[2] = t;
    }
    public String toString(){
	return "Od Node @ " + state[0] + " " + state[1] + " " + state[2];
    }


}
