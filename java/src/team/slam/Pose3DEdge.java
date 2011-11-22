package team.slam;

import april.jmat.Matrix;

/**
 * Used as a prior so we don't have an underconstrained system. Could be used anywhere if
 * we just know where we are at any point.
 */
public class Pose3DEdge extends Edge{

    private Pose3D position;

    public Pose3DEdge(Pose3DNode n1, Pose3D position, Matrix cov){

        nodes = new ArrayList<Pose>();
        nodes.add(n1);

        this.position = position;
        this.cov = cov;

    }

    public int getDOF() {
        return 6;
    }

    public void initialize() {

        Pose3DNode aNode = (Pose3DNode)nodes.get(0);
        if (!aNode.initialized()) {

        }

    }

}
