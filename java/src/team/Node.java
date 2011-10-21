package team;

public abstract class Node{

    private int posInStateVector;

    public abstract double[] getState();

    public int getStateVectorIndex(){
	return posInStateVector;
    }



}