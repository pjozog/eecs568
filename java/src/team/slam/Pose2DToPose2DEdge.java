package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.jmat.CholeskyDecomposition;
import team.common.SixDofCoords;
import team.slam.Linearization;

public class Pose2DToPose2DEdge extends Edge {

    private Pose2D deltaMotion;

    public Pose2DToPose2DEdge(Pose2DNode n1, Pose2DNode n2, Pose2D deltaMotion, Matrix cov) {

        nodes = new ArrayList<Node>();
        nodes.add(n1);
        nodes.add(n2);

        this.deltaMotion = deltaMotion;
        this.cov = cov;

    }


    public int getDOF() {
        return 3;
    }

    /**
     * Takes the deltaMotion and the node that is already initialized (one must be) and
     * makes a prediction about the global position of the other node. This is moving from
     * delta motion to global motion...which is now nicely separated.
     */
    public void initialize() {

        Pose2DNode n1 = (Pose2DNode)nodes.get(0);
        Pose2DNode n2 = (Pose2DNode)nodes.get(1);


        if (n1.isInitialized() && !n2.isInitialized()) {

            double[] prediction = ThreeDofCoords.headToTail(n1.getStateArray(), deltaMotion.getArray());
            n2.init(new Pose2D(prediction));

        } else if (!n1.isInitialized() && n2.isInitialized()) {

            double[] prediction = ThreeDofCoords.headToTail(n2.getStateArray(),
                                                            SixDofCoords.inverse(deltaMotion.getArray()));

            n1.init(new Pose2D(prediction));

        } else (!n1.isInitialized() && !n2.isInitialized()) {
                System.out.println("One of the nodes has to have a value to make a prediction!");
                assert(false);
            }

    }

    protected Matrix getJacobian() {

        return new Matrix(ThreeDofCoords.tailToTailJacob(nodes.get(0).getLinearizationState(),
                                                         nodes.get(1).getLinearizationState()));

    }

    public double[] getResidual() {

        double[] predictedOdom = ThreeDofCoords.tailToTail(nodes.get(0).getLinearizationState(),
                                                           nodes.get(1).getLinearizationState());

        double[] residual = LinAlg.subtract(deltaMotion.getArray(), predictedOdom);


        residual[2] = MathUtil.mod2pi(residual[2]);

        return residual;

    }

    public double[] getChi2Error() {

        double[] predictedOdom = ThreeDofCoords.tailToTail(nodes.get(0).getStateArray(),
                                                           nodes.get(1).getStateArray());

        double[] residual = LinAlg.subtract(deltaMotion.getArray(), predictedOdom);

        residual[2] = MathUtil.mod2pi(residual[2]);

        if (cholInvCov == null) {
            // Create cholInvCov
            CholeskyDecomposition myDecomp = new CholeskyDecomposition(cov.inverse());
            cholInvCov = myDecomp.getL().transpose();
        }

        return LinAlg.matrixAB(cholInvCov.copyArray(), residual);

    }

}
