package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import team.common.SixDofCoords;

public class Pose3DToPose3DEdge extends Edge{

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

    public void initialize() {

        Pose3DNode n1 = (Pose3DNode)nodes.get(0);
        Pose3DNode n2 = (Pose3DNode)nodes.get(1);

        if (n1.isInitialized() && !n2.isInitialized()) {

            Pose3D n1State = n1.getState();
            double[] prediction = SixDofCoords.headToTail(n1State.getArray(), deltaMotion.getArray());
            n2.init(new Pose3D(prediction));

        } else if (!n1.isInitialized() && n2.isInitialized()) {

            Pose3D n2State = n2.getState();
            double[] prediction = SixDofCoords.headToTail(n2State.getArray(), inverse(measure.getArray()));
            n1.init(new Pose3D(prediction));

        } else {
            System.out.println("One of the nodes has to have a value to make a prediction!");
        }


    }

}
