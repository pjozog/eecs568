package team.slam;

import java.util.List;
import java.util.ArrayList;

import april.jmat.Matrix;
import april.jmat.LinAlg;
import april.jmat.CholeskyDecomposition;

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
     * The upper triangular result of the cholesky decomposition of the inverse of the
     * covariance. Phew. Needs to be created during initialization (but thankfully, only once).
     */
    protected Matrix cholInvCov = null;

    /**
     * Degrees of Freedom in the Edge.
     */
    abstract public int getDOF();


    /**
     * Sets up nodes by transforming local measurements to global state-vector worthy measurements.
     */
    abstract public void initialize();


    /**
     * Returns a matrix containing the jacobian of the measurement
     * model for the nodes. It will be evaluated at the current state
     * vector
     */
    abstract protected Matrix getJacobian();


    /**
     * Get the residual
     *
     * Returns the (observation) - (predicted observation).  The
     * predicted observation is from the measurement model.  This is
     * the function we take the jacobian of in getJacobian()
     */
    abstract public double[] getResidual();

    /**
     * Get all the fun stuff about this edge like jacobian blocks and the residual.
     */
    public Linearization getLinearization() {

        Linearization result = new Linearization();

        if (cholInvCov == null) {
            // Create cholInvCov
            CholeskyDecomposition myDecomp = new CholeskyDecomposition(cov.inverse());
            cholInvCov = myDecomp.getL().transpose();
        }

        Matrix jacobians = getJacobian();

        // Incorporate the covarance
        jacobians = cholInvCov.times(jacobians);

        // Split up the jacob into appropriate blocks
        int col = 0;
        for (Node aNode: nodes) {
            double[][]  nodeJBlock = jacobians.copyArray(0, col, getDOF(), aNode.getDOF());
            col += aNode.getDOF();
            result.J.add(nodeJBlock);
        }

        // Incorporate the covarance
        result.residual = LinAlg.matrixAB(cholInvCov.copyArray(), getResidual());

        return result;
    }

    public List<Node> getNodes() {
        return nodes;
    }

}
