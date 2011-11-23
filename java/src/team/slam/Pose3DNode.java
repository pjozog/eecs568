package team.slam;


public class Pose3DNode extends Node {

    Pose3D origState;
    Pose3D currState;

    public Pose3DNode(){

    }

    public int getDOF() {
        return 6;
    }

    public void init(Pose3D aPrediction) {

        initialized = true;

        origState = new Pose3D(aPrediction);
        currState = new Pose3D(aPrediction);

    }

    public Pose3D getState() {
        return currState;
    }

    public Pose3D getOrigState() {
        return origState;
    }

}
