package team.slam;


public class Pose3DNode extends Node {

    Pose3D currState;

    public Pose3DNode(){

    }

    public int getDOF() {
        return 6;
    }

    public void init(Pose3D aPrediction) {

        initialized = true;
        currState = new Pose3D(aPrediction);

    }

    public Pose3D getState() {
        return currState;
    }

    public double[] getStateArray() {
        return currState.getArray();
    }

    public void setStateArray(double[] values) {
        assert(values.length == 6);

        currState.setStateArray(values);

    }

}
