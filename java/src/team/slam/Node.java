package team.slam;

public abstract class Node {

    protected boolean initialized = false;

    abstract public int getDOF();

    // Sory Bjarne
    // abstract public void init(Object aPrediction);

    abstract public double[] getLinearizationState();

    public boolean isInitialized() {
        return initialized;
    }

}
