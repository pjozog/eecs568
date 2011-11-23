package team.slam;

import java.util.List;
import java.util.ArrayList;

public class BackEnd{

    private List<Node> nodes;
    private List<Edge> edges;


    // Book-keeping
    private int nodeDimension;
    private int edgeDimension;
    private int numNewRows;
    private int numNewMeasurements;

    /**
     * Default constructor. Just ivar initializations.
     */
    public BackEnd() {

        nodes = new ArrayList<Node>();
        edges = new ArrayList<Edge>();

        nodeDimension      = 0;
        edgeDimension      = 0;
        numNewRows         = 0;
        numNewMeasurements = 0;

    }

    /**
     * Add the node to our list. This assumes that the front end will only call this
     * function if the node is not already a part of the graph.
     */
    public void addNode(Node node) {

        nodes.add(node);

        nodeDimension += node.getDOF();

    }

    /**
     * Add the edge to our list. Book-keeping and initialization?
     */
    public void addEdge(Edge edge) {

        // Fills in connected nodes appropriately
        edge.initialize();

        edges.add(edge);

        edgeDimension += edge.getDOF();

        numNewRows += edge.getDOF();
        numNewMeasurements++;

    }

    /**
     * Find the least sqaures solution of the system. This will construct the Jacabian and
     * evaluate it at the best state vector estimate, assemble JtSigmaJ, compute the
     * residual, and find the new optimal stuff.
     */
    public boolean solve() {

        // yay!
        return true;
    }

    //TODO: Accessors to get solution. Depends on how we want to draw things.



}
