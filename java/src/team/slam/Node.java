package team.slam;

public abstract class Node {

    protected boolean initialized = false;

    abstract public int getDOF();

    abstract public double[] getLinearizationState();

    abstract public double[] getStateArray();

    public boolean isInitialized() {
        return initialized;
    }

}
