package team.slam;


public class Pose3DNode extends Node {

    private Pose3D currState;
    private Pose3D linearizationState;

    public Pose3DNode(){

    }


    public void init(Pose3D aPrediction) {

        initialized = true;
        currState = new Pose3D(aPrediction);
        linearizationState = new Pose3D(aPrediction);

    }

    public void updateLinearizationPoint() {
        linearizationState = new Pose3D(currState);
    }

    public int getDOF() {
        return 6;
    }

    public Pose3D getState() {
        return currState;
    }

    public double[] getStateArray() {
        return currState.getArray();
    }

    public double[] getLinearizationState() {
        return linearizationState.getArray();
    }

    public void setStateArray(double[] values) {
        assert(values.length == 6);
        currState.setStateArray(values);
    }

}
