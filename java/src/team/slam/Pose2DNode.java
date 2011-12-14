package team.slam;


public class Pose2DNode extends Node {

    private Pose2D currState;
    private Pose2D linearizationState;

    public Pose2DNode(){

    }


    public void init(Pose2D aPrediction) {

        initialized = true;
        currState = new Pose2D(aPrediction);
        linearizationState = new Pose2D(aPrediction);

    }

    public void updateLinearizationPoint() {
        linearizationState = new Pose2D(currState);
    }

    public int getDOF() {
        return 3;
    }

    public Pose2D getState() {
        return currState;
    }

    public double[] getStateArray() {
        return currState.getArray();
    }

    public double[] getLinearizationState() {
        return linearizationState.getArray();
    }

    public void setStateArray(double[] values) {
        assert(values.length == 3);
        currState.setStateArray(values);
    }

}
