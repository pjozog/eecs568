package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;

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


    public Matrix assembleJacobian() {

        if (edgeDimension < nodeDimension) {
            System.out.println("How dare you work with an underconstrained system...");
        }

        Matrix bigJ = new Matrix(edgeDimension, nodeDimension, Matrix.SPARSE);

        int rowIndex = 0;

        // Loop over all the edges
        for (int i = 0; i < edges.size(); i++) {

            Edge anEdge = edges.get(i);

            // Get the jacobian blocks for the edge (numerically or symbolically)
            Linearization edgeLin = getLinearization();

            // For every jacobian block associated with the edge...
            for (int j = 0; i < edgeLin.J.size(); j++) {

                // The starting column index is the state vector index of the associated node
                int colIndex =  anEdge.getNodes().get(j).getIndex();
                bigJ.set(rowIndex, colIndex, jBlock);

            }

            rowIndex += anEdge.getDOF();

        }

        return bigJ;

    }

    private double[] getStateEstimate() {

        int index = 0;

        double[] estimate = new double[nodeDimension];

        // Every node contributes to the state estimate
        for (Node aNode : nodes) {

            double[] nodeState = aNode.getStateArray();
            for (int i=0; i < aNode.getDOF(); i++) {
                estimate[index+i] = nodeState[i];
            }

            index += aNode.getDOF();

        }

        return estimate;
    }

    private void updateStarts(){
        int curIndex = 0;
        for(Node node : nodes){
            node.setIndex(curIndex);
            curIndex += node.getDOF();
        }
    }

}
