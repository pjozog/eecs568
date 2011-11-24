package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import april.jmat.LinAlg;
import april.jmat.CholeskyDecomposition;


// Experimental
import april.jmat.ordering.*;
import team.common.*;

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

        // gaussNewton();
        fasterGaussNewton();
        // experimentalFactoringGausssNewton(new MinimumDegreeOrdering());
        // experimentalFactoringGausssNewton(new SimpleDegreeOrdering());

    }


    /**
     * Directly assemble A and b instead of assebling from J, and InvCov.
     * I did this in PS2, but it was a total mess. Don't even look at it. With our
     * massive refactoring and now knowing what what april.graph does, we can do this in a much
     * more slick way. If you do go through this, it is now OK to look at PS2 and laugh
     * yourself into a better mood.
     */
    private double[] fasterGaussNewton() {

        updateNodeIndices();

        int numIter = 0;

        double[] x = getStateEstimate();

        double maxChange = 0;


        do {

            Matrix A = new Matrix(nodeDimension, nodeDimension, Matrix.SPARSE);
            Matrix b = new Matrix(nodeDimension, 1);


            // Loop over all the edges to add their contributions to A and b
            for (Edge anEdge : edges) {

                // Get the jacobian blocks for the edge (numerically or symbolically)
                Linearization edgeLin = anEdge.getLinearization();

                // For every node associated with the edge
                for (int i = 0; i < anEdge.getNodes().size(); i++) {

                    // getIndex() represents the state vector index of the associated node
                    int aIndex = anEdge.getNodes().get(i).getIndex();

                    double[][] JatW = LinAlg.matrixAtB(edgeLin.J.get(i), edgeLin.cov);

                    // For every node associated with the edge (again)
                    for (int j = 0; j < anEdge.getNodes().size(); j++) {

                        int bIndex =  anEdge.getNodes().get(j).getIndex();

                        double[][] JatWJb = LinAlg.matrixAB(JatW, edgeLin.J.get(j));

                        // Add the contribution of node j and i to A
                        A.plusEquals(aIndex, bIndex, JatWJb);
                    }

                    double[] JatWr = LinAlg.matrixAB(JatW, edgeLin.residual);
                    b.plusEqualsColumnVector(aIndex, 0, JatWr);

                }

            }


            // Tikhanoff regulaization
            A = A.plus(Matrix.identity(A.getRowDimension(), A.getColumnDimension()).times(lambda));
            assert(A.isSparse());

            CholeskyDecomposition myDecomp = new CholeskyDecomposition(A);

            double[] deltaX = myDecomp.solve(b).copyAsVector();

            maxChange = LinAlg.max(LinAlg.abs(deltaX));

            // System.out.println("Max change from guass newton is" + maxChange);

            x = LinAlg.add(x, deltaX);

            updateNodesWithNewState(x);

            numIter++;

        } while (numIter < maxIter && maxChange > epsilon);

        return x;

    }


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

            // System.out.println("Max change from guass newton is" + maxChange);

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


    public List<Node> getNodes() {
        return nodes;
    }

    public List<Edge> getEdges() {
        return edges;
    }


    //////////////////////////////
    ////// EXPERIMENTAL
    ////// This code is barely intelligible
    //////////////////////////////

    public int experimentalNodeArrayIndex(Node aNode) {

        int curIndex = 0;
        for(Node node : nodes){
            if (node == aNode) {
                return curIndex;
            }
            curIndex++;
        }

        return -1;

    }

    public Matrix experiemenalMakeSymbolicA()
    {
        Matrix A = new Matrix(nodes.size(), nodes.size(), Matrix.SPARSE);

        for (Edge anEdge : edges) {

            List<Node> theNodes = anEdge.getNodes();

            for (int i = 0; i < theNodes.size(); i++) {
                for (int j = 0; j < theNodes.size(); j++) {
                    int indexOne = experimentalNodeArrayIndex(theNodes.get(i));
                    int indexTwo = experimentalNodeArrayIndex(theNodes.get(j));

                    if (indexOne == -1 || indexTwo == -1) {
                        System.out.println("EXPERIMENTAL ERROR: Node not found in node array!");
                    }

                    A.set(indexOne, indexTwo, 1);
                }
            }
        }

        return A;
    }

    private double[] experimentalFactoringGausssNewton(Ordering ordering) {

        updateNodeIndices();

        int numIter = 0;

        double[] x = getStateEstimate();

        double maxChange = 0;

        do {

            Matrix A = new Matrix(nodeDimension, nodeDimension, Matrix.SPARSE);
            Matrix b = new Matrix(nodeDimension, 1);


            // Loop over all the edges to add their contributions to A and b
            for (Edge anEdge : edges) {

                // Get the jacobian blocks for the edge (numerically or symbolically)
                Linearization edgeLin = anEdge.getLinearization();

                // For every node associated with the edge
                for (int i = 0; i < anEdge.getNodes().size(); i++) {

                    // getIndex() represents the state vector index of the associated node
                    int aIndex = anEdge.getNodes().get(i).getIndex();

                    double[][] JatW = LinAlg.matrixAtB(edgeLin.J.get(i), edgeLin.cov);

                    // For every node associated with the edge (again)
                    for (int j = 0; j < anEdge.getNodes().size(); j++) {

                        int bIndex =  anEdge.getNodes().get(j).getIndex();

                        double[][] JatWJb = LinAlg.matrixAB(JatW, edgeLin.J.get(j));

                        // Add the contribution of node j and i to A
                        A.plusEquals(aIndex, bIndex, JatWJb);
                    }

                    double[] JatWr = LinAlg.matrixAB(JatW, edgeLin.residual);
                    b.plusEqualsColumnVector(aIndex, 0, JatWr);

                }

            }


            // Tikhanoff regulaization
            A = A.plus(Matrix.identity(A.getRowDimension(), A.getColumnDimension()).times(lambda));
            assert(A.isSparse());


            Matrix SA = experiemenalMakeSymbolicA();

            int saPerm[] = ordering.getPermutation(SA);

            int perm[] = new int[nodeDimension];
            int pos = 0;

            // updateNodeIndices();
            // System.out.println("Node state index info");
            // for (Node aNode : nodes) {
            //     System.out.println(aNode.getIndex());
            // }


            // System.out.println("saPerm info "+ saPerm.length);
            // ArrayUtil.print1dArray(saPerm);

            for (int saidx = 0; saidx < saPerm.length; saidx++) {
                int gnidx = saPerm[saidx];
                Node gn = nodes.get(gnidx);
                int gnpos = gn.getIndex();

                // System.out.println("ASDF Info "+gnidx+ " "+gnpos+" "+gn.getDOF());

                for (int i = 0; i < gn.getDOF(); i++)
                    perm[pos++] = gnpos + i;
            }

            Matrix PAP = A.copyPermuteRowsAndColumns(perm);
            Matrix PB = b.copy();
            PB.permuteRows(perm);


            double acc = 0;
            int count = 0;
            boolean underconstrained = false;

            for (int i = 0; i < PAP.getRowDimension(); i++) {
                if (PAP.get(i,i) == 0) {
                    underconstrained = true;
                } else {
                    acc += PAP.get(i,i);
                    count ++;
                }
            }

            if (underconstrained) {
                System.out.println("CholeskySolver: underconstrained graph. Trying to fix it (hack!)");

                if (count == 0) {
                    acc = 1;
                    count = 1;
                }


                // Tikhanoff regulaization
                // PAP = PAP.plus(Matrix.identity(PAP.getRowDimension(), PAP.getColumnDimension()).times(lambda));
                // assert(A.isSparse());

                for (int i = 0; i < PAP.getRowDimension(); i++) {
                    if (PAP.get(i,i) == 0) {
                        PAP.set(i,i, acc / count);
                    }

                    assert(PAP.get(i,i) > 0); //System.out.printf("%10d %15f\n", i, PAP.get(i,i));
                }
            }

            CholeskyDecomposition cd = new CholeskyDecomposition(PAP);
            // L = cd.getL();
            Matrix deltaX = cd.solve(PB);
            // System.out.println("perm info "+ perm.length);
            // ArrayUtil.print1dArray(perm);



            // double[] deltaXVecBefore = deltaX.copyAsVector();
            deltaX.inversePermuteRows(perm);
            // System.out.println("SA dim "+ SA.getRowDimension() +" "+ SA.getColumnDimension());
            // System.out.println("deltaX dim "+ deltaX.getRowDimension() +" "+ deltaX.getColumnDimension());

            double[] deltaXVec = deltaX.copyAsVector();

            maxChange = LinAlg.max(LinAlg.abs(deltaXVec));

            // System.out.println("Max change from guass newton is" + maxChange);

            x = LinAlg.add(x, deltaXVec);

            updateNodesWithNewState(x);

            numIter++;

            // System.out.println("Node state index info");
            // for (Node aNode : nodes) {
            //     System.out.println(aNode.getIndex());
            // }


        } while (numIter < maxIter && maxChange > epsilon);

        return x;

    }


}
