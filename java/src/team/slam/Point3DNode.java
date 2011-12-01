package team.slam;

public class Point3DNode extends Node {

    private Point3D currState;
    private Point3D linearizationState;

    private int id;

    public Point3DNode(){

    }

    public Point3DNode(int id) {
        this.id = id;
    }


    public void init(Point3D aPrediction) {

        initialized = true;
        currState = new Point3D(aPrediction);
        linearizationState = new Point3D(aPrediction);

    }


    public void updateLinearizationPoint() {
        linearizationState = new Point3D(currState);
    }

    public int getDOF() {
        return 3;
    }

    public Point3D getState() {
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
        assert(values.length == 3);
        currState.setStateArray(values);
    }

}
