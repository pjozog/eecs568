/** EECS 568 Homework 2, SLAM
Steve Chaves
Schuyler Cohen
Patrick O'Keefe
Paul Ozog
**/

package team.PS2;

import java.util.*;
import april.config.*;
import april.sim.*;

public abstract class Edge{

    public Node node1;
    public Node node2;

    public static Config config;

    protected int jacobianStartRow;
    protected int block1Column;
    protected int block2Column;

    private OldSimulator.odometry_t myOdom;

    public abstract JacobBlock getJacob(List<Node> theStateVector);

    public abstract double [] getResiduals();

    public abstract CovBlock getCovBlock(double t_l, double t_r);

    public void setOdom(OldSimulator.odometry_t odo) {
        myOdom = odo;
    }

    public OldSimulator.odometry_t getOdom() {
        return myOdom;
    }

    public boolean isLandmarkEdge() {
        return node1.isLand() || node2.isLand();
    }

    public int getJacobianStartRow() {
        return jacobianStartRow;
    }

}

