package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import april.jmat.LinAlg;
import april.jmat.CholeskyDecomposition;

/**
 * Used as a prior so we don't have an underconstrained system. Could be used anywhere if
 * we just know where we are at any point.
 */
public class Pose2DEdge extends Edge{

    private Pose2D position;

    public Pose2DEdge(Pose2DNode n1, Pose2D position, Matrix cov){

        nodes = new ArrayList<Node>();
        nodes.add(n1);

        this.position = position;
        this.cov = cov;

    }

    public int getDOF() {
        return 3;
    }

    public void initialize() {
        // Initialize connected nodes
        Pose2DNode aNode = (Pose2DNode)nodes.get(0);
        if (!aNode.isInitialized()) {
            aNode.init(position);
        }
    }


    protected Matrix getJacobian() {

        return Matrix.identity(3,3);

    }

    public double[] getResidual() {

        double[] predicted = nodes.get(0).getLinearizationState();
        double[] residual = LinAlg.subtract(position.getArray(), predicted);

        return residual;

    }

    public double[] getChi2Error() {


        if (cholInvCov == null) {
            // Create cholInvCov
            CholeskyDecomposition myDecomp = new CholeskyDecomposition(cov.inverse());
            cholInvCov = myDecomp.getL().transpose();
        }


        double[] predicted = nodes.get(0).getStateArray();
        double[] residual = LinAlg.subtract(position.getArray(), predicted);

        return LinAlg.matrixAB(cholInvCov.copyArray(), residual);
    }

}
