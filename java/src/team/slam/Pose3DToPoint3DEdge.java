package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import april.jmat.LinAlg;
import team.common.SixDofCoords;

public class Pose3DToPoint3DEdge extends Edge {

    private Point3D observation;

    /**
     * @param n1 Pose from which point was observed
     * @param n2 The point node that is observed
     * @param observation What the sensors are telling us relative to n1
     * @param cov uncertain in observation
     */
    public Pose3DToPoint3DEdge(Pose3DNode n1, Point3DNode n2, Point3D observation, Matrix cov){

        nodes = new ArrayList<Node>();
        nodes.add(n1);
        nodes.add(n2);

        this.observation = observation;
        this.cov = cov;

    }

    public int getDOF() {
        return 3;
    }

    /**
     * Transforms the observation (which is relative to the pose) into global coordinates
     * and initialzes the point node with that information.
     */
    public void initialize() {

        Pose3DNode n1 = (Pose3DNode)nodes.get(0);
        Point3DNode n2 = (Point3DNode)nodes.get(1);

        if (!n1.isInitialized()) {
            System.out.println("Good God! Trying to initialize a 3d point without a pose estimate?!?");
            assert(false);
        }

        if (!n2.isInitialized()) {

            // Treat observation as a Pose3D
            Pose3D fakeDeltaPose = new Pose3D();
            fakeDeltaPose.setX(observation.getX());
            fakeDeltaPose.setY(observation.getY());
            fakeDeltaPose.setZ(observation.getZ());

            // double[] prediction = SixDofCoords.headToTail(n1.getLinearizationState(), fakeDeltaPose.getArray());
            double[] prediction = SixDofCoords.headToTail(n1.getStateArray(), fakeDeltaPose.getArray());

            // Only keep the 3 important guys
            double[] slimPrediction = new double[] {prediction[0], prediction[1], prediction[2]};
            n2.init(new Point3D(slimPrediction));

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

        //We want the 3x9 jacobian that relates Pose3D's global 6DOF
        //vector and Point3D's global 3DOF vector to the x,y,z of the
        //point in the pose's frame.
        return new Matrix(JFull.copyArray(0, 0, 3, 9));

    }

    //Must be 3x1 vector (xyz)
    public double[] getResidual() {

        // double[] pointEst = nodes.get(1).getStateArray();
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

}
