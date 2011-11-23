package team.slam;

public class Point3DNode extends Node {

    Point3D origState;
    Point3D currState;

    public Point3DNode(){

    }

    public int getDOF() {
        return 3;
    }

    public Point3D getState() {
        return currState;
    }

    public Point3D getOrigState() {
        return origState;
    }

    public void init(Point3D aPrediction) {
        origState = new Point3D(aPrediction);
        currState = new Point3D(aPrediction);
    }

}
