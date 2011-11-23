package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
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

            Pose3D n1State = n1.getState();
            double[] prediction = SixDofCoords.headToTail(n1State.getArray(), deltaMotion.getArray());
            n2.init(new Pose3D(prediction));

        } else if (!n1.isInitialized() && n2.isInitialized()) {

            Pose3D n2State = n2.getState();
            double[] prediction = SixDofCoords.headToTail(n2State.getArray(),
                                                          SixDofCoords.inverse(deltaMotion.getArray()));
            n1.init(new Pose3D(prediction));

        } else {
            System.out.println("One of the nodes has to have a value to make a prediction!");
        }

    }

    protected Matrix getJacobian(List<double[]> linPoints) {

        return new Matrix(SixDofCoords.headToTailJacob(linPoints.get(0), linPoints.get(1)));

    }

    protected double[] getResidual() {
        return new double[] {0.0};
    }




}