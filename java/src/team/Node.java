package team;

public abstract class Node{

    private int posInStateVector;

    protected double[] state;

    public  double[] getState(){
	return state;
    }
    
    public boolean isLand(){
	return state.length == 2;
    }

    /**Explicit position of the first variable in state vector**/
    public int getStateVectorIndex(){
	return posInStateVector;
    }

    public abstract String toString();


}