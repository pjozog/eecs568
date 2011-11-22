package team.slam;

public abstract class Node{

    private double [] state;
    protected boolean isInitialized;

    abstract public int getDOF();

    public boolean initialized() {
        return isInitialized;
    }

}
