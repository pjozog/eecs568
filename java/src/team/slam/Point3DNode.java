package team.slam;

public class Point3DNode extends Node {

    Point3D linState;
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

    public double[] getLinearizationState() {
        return linState.getArray();
    }

    public void init(Point3D aPrediction) {
        linState = new Point3D(aPrediction);
        currState = new Point3D(aPrediction);
    }

}
