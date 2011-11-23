package team.slam;

import april.jmat.*;

public class SlamExample {
    
    public static void main(String[] args) {

        //Create the backend solver
        BackEnd backend      = new BackEnd();

        //Create the origin
        Pose3DNode origin    = new Pose3DNode();
        Pose3D prior         = new Pose3D(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        Pose3DEdge priorEdge = new Pose3DEdge(origin, prior, Matrix.identity(6,6));

        //Add some points
        Pose3DNode pose1     = new Pose3DNode();
        Pose3D     odom1     = new Pose3D(new double[]{1,2,3,-.1,-.2,1.3});

        Pose3DNode pose2     = new Pose3DNode();
        Pose3D     odom2     = new Pose3D(new double[]{1,2,3,-.1,-.2,1.3});

        Pose3DToPose3DEdge edge01    = new Pose3DToPose3DEdge(origin, pose1, odom1, Matrix.identity(6,6));
        Pose3DToPose3DEdge edge12    = new Pose3DToPose3DEdge(pose1, pose2, odom2, Matrix.identity(6,6));

        backend.addNode(origin);
        backend.addNode(pose1);
        backend.addNode(pose2);

        backend.addEdge(priorEdge);
        backend.addEdge(edge01);
        backend.addEdge(edge12);

        backend.solve();

    }

}
