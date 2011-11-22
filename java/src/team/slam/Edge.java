package team.slam;

public abstract class Edge {

    /**
     * The list of nodes in the edge. It's a list because a PoseEdge will only have one
     * node. I suppose an Edge can have an arbitrary number of nodes it represents.
     */
    protected List<Node> nodes;


    /**
     * Degrees of Freedom in the Edge.
     */
    abstract int getDOF();


    /**
     * Will give the connected nodes appropriate values and also inform them of itself.
     * Consfusing? I agree.
     */
    abstract void initialize();


}
