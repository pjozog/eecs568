package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import april.jmat.LinAlg;
import april.jmat.CholeskyDecomposition;

public class BackEnd{

    private List<Node> nodes;
    private List<Edge> edges;

    // Tunable parameters
    double lambda = 1.0;
    double epsilon = .0001;
    double maxIter = 10;


    // Book-keeping
    private int nodeDimension;
    private int edgeDimension;
    private int numNewRows;
    private int numNewMeasurements;

    /**
     * Default constructor. Just ivar initializations.
     */
    public BackEnd() {
        init();
    }

    public BackEnd(int numIter, double l, double e){
        init();
        maxIter = numIter;
        epsilon = e;
        lambda = l;
    }

    private void init(){

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
    public void solve() {

        gaussNewton();

    }


    //TODO: Accessors to get solution. Depends on how we want to draw things.


    private double[] gaussNewton() {

        int numIter = 0;

        double[] x = getStateEstimate();

        double maxChange = 0;

        Matrix sigmaInv = assembleInvCov();

        do {

            Matrix J = assembleJacobian();
            double[] residuals = assembleResiduals();

            Matrix jtSig = J.transpose().times(sigmaInv);

            Matrix A = jtSig.times(J);

            Matrix b = jtSig.times(Matrix.columnMatrix(residuals));

            // Tikhanoff regulaization
            A = A.plus(Matrix.identity(A.getRowDimension(), A.getColumnDimension()).times(lambda));
            assert(A.isSparse());

            CholeskyDecomposition myDecomp = new CholeskyDecomposition(A);

            double[] deltaX = myDecomp.solve(b).copyAsVector();

            maxChange = LinAlg.max(LinAlg.abs(deltaX));

            x = LinAlg.add(x, deltaX);

            updateNodesWithNewState(x);

            numIter++;

        } while (numIter < maxIter && maxChange > epsilon);

        return x;

    }


    private double[] assembleResiduals() {

        double[] theResiduals = new double[edgeDimension];

        int count = 0;
        for (Edge anEdge : edges) {

            double[] edgeResid = anEdge.getResidual();

            for (int i = 0; i < anEdge.getDOF(); i++) {
                theResiduals[count+i] = edgeResid[i];
            }

            count += anEdge.getDOF();

        }

        return theResiduals;

    }




    private Matrix assembleJacobian() {

        updateNodeIndices();

        if (edgeDimension < nodeDimension) {
            System.out.println("How dare you work with an underconstrained system...");
            assert(false);
        }

        Matrix bigJ = new Matrix(edgeDimension, nodeDimension, Matrix.SPARSE);

        int rowIndex = 0;

        // Loop over all the edges
        for (int i = 0; i < edges.size(); i++) {

            Edge anEdge = edges.get(i);

            // Get the jacobian blocks for the edge (numerically or symbolically)
            Linearization edgeLin = anEdge.getLinearization();

            // For every jacobian block associated with the edge...
            for (int j = 0; j < edgeLin.J.size(); j++) {

                // The starting column index is the state vector index of the associated node
                int colIndex =  anEdge.getNodes().get(j).getIndex();
                double [][] jBlock = edgeLin.J.get(j);
                bigJ.set(rowIndex, colIndex, jBlock);

            }

            rowIndex += anEdge.getDOF();

        }

        return bigJ;

    }


    /*Takes all the cov blocks from the edges and assembles the large inverse matrix*/
    private Matrix assembleInvCov(){

        int curIndex = 0;

        /*edgeDimension is the total DOF of all edges*/
        Matrix cov = new Matrix(edgeDimension, edgeDimension, Matrix.SPARSE);
        for(Edge edge : edges){

            double[][] covInv = edge.getCov().inverse().copyArray();

            cov.set(curIndex, curIndex, covInv);

            curIndex += edge.getDOF();
        }
        return cov;

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


    private void updateNodesWithNewState(double[] x) {

        int count = 0;
        for (Node aNode : nodes) {

            int nodeDim = aNode.getDOF();

            double[] newState = new double[nodeDim];
            for (int i = 0; i < nodeDim; i++) {
                newState[i] = x[count + i];
            }

            aNode.setStateArray(newState);

            count += nodeDim;
        }

    }


    private void updateNodeIndices(){
        int curIndex = 0;
        for(Node node : nodes){
            node.setIndex(curIndex);
            curIndex += node.getDOF();
        }
    }



}
