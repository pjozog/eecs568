package team.slam;

public class Point3DNode extends Node {

    Point3D currState;

    public Point3DNode(){

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

    public void setStateArray(double[] values) {
        assert(values.length == 3);

        currState.setStateArray(values);

    }

    public void init(Point3D aPrediction) {

        initialized = true;
        currState = new Point3D(aPrediction);

    }

}
