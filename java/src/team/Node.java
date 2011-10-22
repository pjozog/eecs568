package team;

import java.util.*;


public abstract class Node{

    /*this is the index of the node in List<Node>*/
    protected int posInStateVector;

    /*this is the index of the first element in the absolute state vector*/
    protected int absIndexInStateVector;
    protected double[] state;



    /*index of node in state vector*/
    protected ArrayList<Integer>landmarks = new ArrayList<Integer>();

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

    /**index of node in state vector**/
    public ArrayList<Integer> getLandmarksSeen(){
        return landmarks;
    }
    public void sawLandmark(Integer i){
        landmarks.add(i);
    }

    public abstract String toString();


    public int addToState(double [] deltaX, int index) {

        for (int i = 0 ; i < stateLength() ; i++) {
            state[i] += deltaX[index+i];
        }

        return stateLength();

    }

}