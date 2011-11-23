package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import team.slam.Linearization;

public abstract class Edge {

    /**
     * The list of nodes in the edge. It's a list because a PoseEdge will only have one
     * node. I suppose an Edge can have an arbitrary number of nodes it represents.
     */
    protected List<Node> nodes;

    /**
     * The uncertainty of the edge.
     */
    protected Matrix cov;

    /**
     * Degrees of Freedom in the Edge.
     */
    abstract public int getDOF();


    /**
     * Sets up nodes by transforming local measurements to global state-vector worthy measurements.
     */
    abstract public void initialize();


    /**
     * Returns a matrix containing the jacobian blocks for the nodes. The points to
     * evaluate at for each node are passed in.
     */
    abstract prote Matrix getJacobian(List<double[]> linPoints);


    /**
     * Get the residual
     *
     * TODO: I haven't thought about this yet.
     */
    abstract protected double[] getResidual();


    /**
     * Get all the fun stuff about this edge like jacobian blocks and the residual.
     */
    public Linearization getLinearization() {

        Linearization result = new Linearization();

        // Accumulate the places to linearize our shtuff
        List<double[]> linearizationPoints = new ArrayList<double[]>();
        for (Node aNode : nodes) {
            linearizationPoints.add(aNode.getLinearizationState());
        }

        Matrix jacobians = getJacobian(linearizationPoints);

        // Split up the jacob into appropriate blocks
        int col = 0;
        for (Node aNode: nodes) {
            double[][]  nodeJBlock = jacobians.copyArray(0, col, getDOF(), aNode.getDOF());
            col += aNode.getDOF();
            result.J.add(nodeJBlock);
        }

        result.residual = getResidual();

        return result;
    }

}
