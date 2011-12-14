package team.slam;

public class Point2DNode extends Node {

    private Point2D currState;
    private Point2D linearizationState;

    private int id;

    public Point2DNode(){

    }

    public Point2DNode(int id) {
        this.id = id;
    }


    public void init(Point2D aPrediction) {
        initialized = true;
        currState = new Point2D(aPrediction);
        linearizationState = new Point2D(aPrediction);

    }

    public void updateLinearizationPoint() {
        linearizationState = new Point2D(currState);
    }

    public int getDOF() {
        return 2;
    }

    public Point2D getState() {
        return currState;
    }

    public double[] getStateArray() {
        return currState.getArray();
    }

    public double[] getLinearizationState() {
        return linearizationState.getArray();
    }

    public int getId() {
        return id;
    }

    public void setStateArray(double[] values) {
        assert(values.length == 2);
        currState.setStateArray(values);
    }

}
