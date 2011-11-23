package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
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
        }

        if (!n2.isInitialized()) {

            Pose3D n1State = n1.getState();

            // Treat observation as a Pose3D
            Pose3D fakeDeltaPose = new Pose3D();
            fakeDeltaPose.setX(observation.getX());
            fakeDeltaPose.setY(observation.getY());
            fakeDeltaPose.setZ(observation.getZ());

            double[] prediction = SixDofCoords.headToTail(n1State.getArray(), fakeDeltaPose.getArray());

            // Only keep the 3 important guys
            double[] slimPrediction = new double[] {prediction[0], prediction[1], prediction[2]};
            n2.init(new Point3D(slimPrediction));

        }

    }

}
