package team;

public abstract class Node{

    /*this is the index of the node in List<Node>*/
    protected int posInStateVector;

    /*this is the index of the first element in the absolute state vector*/
    protected int absIndexInStateVector;
    protected double[] state;

    public  double[] getState(){
	return state;
    }
    
    public int stateLength(){
	return state.length;
    }

    public boolean isLand(){
	return state.length == 2;
    }

    /**Explicit position of the first variable in state vector**/
    public int getStateVectorIndex(){
	return posInStateVector;
    }

    public int getAbsIndex(){
	return absIndexInStateVector;
    }

    public abstract String toString();


}