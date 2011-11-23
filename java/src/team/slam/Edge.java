package team.slam;

import java.util.List;
import april.jmat.Matrix;

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
     * Will give the connected nodes appropriate values and also inform them of itself.
     * Consfusing? I agree.
     */
    abstract public void initialize();


    //TODO: Function to compute residual?


}
