package team.slam;


public class Pose3DNode extends Node {

    Pose3D origState;
    Pose3D currState;

    public Pose3DNode(){

    }

    public int getDOF() {
        return 6;
    }

    public void init(Pose3D aPose) {

        initialized = true;

        origState = new Pose3D(aPose);
        currState = new Pose3D(aPose);

    }

    public Pose3D getState() {
        return currState;
    }

    public Pose3D getOrigState() {
        return origState;
    }

}
