package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.jmat.CholeskyDecomposition;
import team.common.SixDofCoords;
import team.slam.Linearization;

public class Pose3DToPose3DEdge extends Edge {

    private Pose3D deltaMotion;

    public Pose3DToPose3DEdge(Pose3DNode n1, Pose3DNode n2, Pose3D deltaMotion, Matrix cov) {

        nodes = new ArrayList<Node>();
        nodes.add(n1);
        nodes.add(n2);

        this.deltaMotion = deltaMotion;
        this.cov = cov;

    }


    public int getDOF() {
        return 6;
    }

    /**
     * Takes the deltaMotion and the node that is already initialized (one must be) and
     * makes a prediction about the global position of the other node. This is moving from
     * delta motion to global motion...which is now nicely separated.
     */
    public void initialize() {

        Pose3DNode n1 = (Pose3DNode)nodes.get(0);
        Pose3DNode n2 = (Pose3DNode)nodes.get(1);

        if (n1.isInitialized() && !n2.isInitialized()) {

            // double[] prediction = SixDofCoords.headToTail(n1.getLinearizationState(), deltaMotion.getArray());
            double[] prediction = SixDofCoords.headToTail(n1.getStateArray(), deltaMotion.getArray());
            n2.init(new Pose3D(prediction));

        } else if (!n1.isInitialized() && n2.isInitialized()) {

            // double[] prediction = SixDofCoords.headToTail(n2.getLinearizationState(),
                                                          // SixDofCoords.inverse(deltaMotion.getArray()));
            double[] prediction = SixDofCoords.headToTail(n2.getStateArray(),
                                                          SixDofCoords.inverse(deltaMotion.getArray()));

            n1.init(new Pose3D(prediction));

        } else {
            System.out.println("One of the nodes has to have a value to make a prediction!");
            assert(false);
        }

    }

    protected Matrix getJacobian() {

        return new Matrix(SixDofCoords.tailToTailJacob(nodes.get(0).getLinearizationState(),
                                                       nodes.get(1).getLinearizationState()));

    }

    public double[] getResidual() {

        double[] predictedOdom = SixDofCoords.tailToTail(nodes.get(0).getLinearizationState(),
                                                         nodes.get(1).getLinearizationState());

        double[] residual = LinAlg.subtract(deltaMotion.getArray(), predictedOdom);


        residual[3] = MathUtil.mod2pi(residual[3]);
        residual[4] = MathUtil.mod2pi(residual[4]);
        residual[5] = MathUtil.mod2pi(residual[5]);

        return residual;

    }

    public double[] getChi2Error() {

        double[] predictedOdom = SixDofCoords.tailToTail(nodes.get(0).getStateArray(),
                                                         nodes.get(1).getStateArray());

        double[] residual = LinAlg.subtract(deltaMotion.getArray(), predictedOdom);


        residual[3] = MathUtil.mod2pi(residual[3]);
        residual[4] = MathUtil.mod2pi(residual[4]);
        residual[5] = MathUtil.mod2pi(residual[5]);

        if (cholInvCov == null) {
            // Create cholInvCov
            CholeskyDecomposition myDecomp = new CholeskyDecomposition(cov.inverse());
            cholInvCov = myDecomp.getL().transpose();
        }

        return LinAlg.matrixAB(cholInvCov.copyArray(), residual);

    }

}
