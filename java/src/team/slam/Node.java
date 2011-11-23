package team.slam;

public abstract class Node{

    private double [] state;
    protected boolean initialized = false;

    abstract public int getDOF();
    abstract public void init();


    public boolean isInitialized() {
        return initialized;
    }

}
