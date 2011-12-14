package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import april.jmat.LinAlg;
import april.jmat.CholeskyDecomposition;
import team.common.SixDofCoords;

public class Pose2DToPoint2DEdge extends Edge {

    private Point2D observation;

    /**
     * @param n1 Pose from which point was observed
     * @param n2 The point node that is observed
     * @param observation What the sensors are telling us relative to n1
     * @param cov uncertain in observation
     */
    public Pose2DToPoint2DEdge(Pose2DNode n1, Point2DNode n2, Point2D observation, Matrix cov){

        nodes = new ArrayList<Node>();
        nodes.add(n1);
        nodes.add(n2);

        this.observation = observation;
        this.cov = cov;

    }

    public int getDOF() {
        return 2;
    }

    /**
     * Transforms the observation (which is relative to the pose) into global coordinates
     * and initialzes the point node with that information.
     */
    public void initialize() {

        Pose2DNode n1 = (Pose2DNode)nodes.get(0);
        Point2DNode n2 = (Point2DNode)nodes.get(1);

        if (!n1.isInitialized()) {
            System.out.println("Good God! Trying to initialize a 2d point without a pose estimate?!?");
            assert(false);
        }

        if (!n2.isInitialized()) {

            // Treat observation as a Pose2D
            Pose2D fakeDeltaPose = new Pose2D();
            fakeDeltaPose.setX(observation.getX());
            fakeDeltaPose.setY(observation.getY());
            /*TODO HUH*/
            // double[] prediction = SixDofCoords.headToTail(n1.getLinearizationState(), fakeDeltaPose.getArray());
            double[] prediction = SixDofCoords.headToTail(n1.getStateArray(), fakeDeltaPose.getArray());

            // Only keep the 3 important guys
            double[] slimPrediction = new double[] {prediction[0], prediction[1], prediction[2]};
            n2.init(new Point2D(slimPrediction));

        }

    }

    protected Matrix getJacobian() {

        double[] pointEst = nodes.get(1).getLinearizationState();

        double[] fakePose = new double[6];
        fakePose[0] = pointEst[0];
        fakePose[1] = pointEst[1];
        fakePose[2] = pointEst[2];
        fakePose[3] = 0.0;
        fakePose[4] = 0.0;
        fakePose[5] = 0.0;

        Matrix JFull = new Matrix(SixDofCoords.tailToTailJacob(nodes.get(0).getLinearizationState(),
                                                               fakePose));

        //We want the 3x9 jacobian that relates Pose2D's global 6DOF
        //vector and Point2D's global 2DOF vector to the x,y,z of the
        //point in the pose's frame.
        return new Matrix(JFull.copyArray(0, 0, 3, 9));

    }

    //Must be 3x1 vector (xyz)
    public double[] getResidual() {

        double[] pointEst = nodes.get(1).getLinearizationState();

        double[] fakePose = new double[6];
        fakePose[0] = pointEst[0];
        fakePose[1] = pointEst[1];
        fakePose[2] = pointEst[2];
        fakePose[3] = 0.0;
        fakePose[4] = 0.0;
        fakePose[5] = 0.0;

        double[] relPose = SixDofCoords.tailToTail(nodes.get(0).getLinearizationState(),
                                                   fakePose);

        double[] predictedXyz = SixDofCoords.getPosition(relPose);

        double[] residual = LinAlg.subtract(observation.getArray(), predictedXyz);

        return residual;

    }

    public double[] getChi2Error() {

        double[] pointEst = nodes.get(1).getStateArray();

        double[] fakePose = new double[6];
        fakePose[0] = pointEst[0];
        fakePose[1] = pointEst[1];
        fakePose[2] = pointEst[2];
        fakePose[3] = 0.0;
        fakePose[4] = 0.0;
        fakePose[5] = 0.0;

        double[] relPose = SixDofCoords.tailToTail(nodes.get(0).getStateArray(),
                                                   fakePose);

        double[] predictedXyz = SixDofCoords.getPosition(relPose);

        double[] residual = LinAlg.subtract(observation.getArray(), predictedXyz);


        if (cholInvCov == null) {
            // Create cholInvCov
            CholeskyDecomposition myDecomp = new CholeskyDecomposition(cov.inverse());
            cholInvCov = myDecomp.getL().transpose();
        }

        return LinAlg.matrixAB(cholInvCov.copyArray(), residual);

    }

}
