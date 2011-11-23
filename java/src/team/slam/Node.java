package team.slam;

public abstract class Node{

    private double [] state;
    protected boolean initialized = false;

    abstract public int getDOF();

    // Sory Bjarne
    // abstract public void init(Object aPrediction);


    public boolean isInitialized() {
        return initialized;
    }

}
