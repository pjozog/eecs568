package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import april.jmat.LinAlg;


/**
 * Used as a prior so we don't have an underconstrained system. Could be used anywhere if
 * we just know where we are at any point.
 */
public class Pose3DEdge extends Edge{

    private Pose3D position;

    public Pose3DEdge(Pose3DNode n1, Pose3D position, Matrix cov){

        nodes = new ArrayList<Node>();
        nodes.add(n1);

        this.position = position;
        this.cov = cov;

    }

    public int getDOF() {
        return 6;
    }

    public void initialize() {
        // Initialize connected nodes
        Pose3DNode aNode = (Pose3DNode)nodes.get(0);
        if (!aNode.isInitialized()) {
            aNode.init(position);
        }
    }


    protected Matrix getJacobian() {

        return Matrix.identity(6,6);

    }

    public double[] getResidual() {

        double[] predicted = nodes.get(0).getLinearizationState();
        double[] residual = LinAlg.subtract(position.getArray(), predicted);

        return residual;

    }

}
