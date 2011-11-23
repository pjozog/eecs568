package team.slam;

public abstract class Node {

    protected boolean initialized = false;

    private int stateIndex;

    abstract public int getDOF();

    abstract public double[] getLinearizationState();

    abstract public double[] getStateArray();



    public boolean isInitialized() {
        return initialized;
    }

    public void setIndex(int index){
        stateIndex = index;
    }

    public int getIndex(){
        return stateIndex;
    }
}
