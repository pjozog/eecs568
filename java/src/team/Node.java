package team;

public abstract class Node{

    private int posInStateVector;

    protected double[] state;

    public  double[] getState(){
	return state;
    }

    public int getStateVectorIndex(){
	return posInStateVector;
    }

    public abstract String toString();


}