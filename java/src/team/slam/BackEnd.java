package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.config.*;
import april.jmat.*;
import team.common.*;

// Experimental
import april.jmat.ordering.*;
import java.io.FileWriter;
import java.io.BufferedWriter;


public class BackEnd{

    private List<Node> nodes;
    private List<Edge> edges;

    private SparseFactorizationSystem sparseFactor = new SparseFactorizationSystem();

    // Tunable parameters
    private double lambda = 1.0;
    private double epsilon = .0001;
    private double maxIter = 10;
    private boolean useIncremental = true;
    private boolean verbose = false;
    private boolean saveRSparsity = false;

    // Controls the frequency of update types. tunable.
    private int updateRate = 1;
    private int solveRate  = 1;
    private int batchSolveRate = 100000000;

    // Book-keeping
    private int nodeDimension;
    private int edgeDimension;
    private int numNewMeasurements;
    private int numSteps = 0;

    /**
     * Default constructor. Just ivar initializations. Uses the sensible defaults above
     * for the tunable parameters.
     */
    public BackEnd() {
        init();
    }


    /**
     * Constructor that reads tunable parameters from a config file. Usefuly so we don't
     * have to recompile as much.
     */
    public BackEnd(Config config){

        init();

        lambda         = config.requireDouble("simulator.lambda");
        epsilon        = config.requireDouble("simulator.epsilon");
        maxIter        = config.requireInt("simulator.numConverge");

        updateRate     = config.requireInt("simulator.updateRate");
        solveRate      = config.requireInt("simulator.solveRate");
        batchSolveRate = config.requireInt("simulator.batchSolveRate");

        useIncremental = config.requireBoolean("simulator.useIncremental");

    }


    /**
     * Ivar initialization.
     */
    private void init(){

        nodes = new ArrayList<Node>();
        edges = new ArrayList<Edge>();

        nodeDimension      = 0;
        edgeDimension      = 0;
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

        numNewMeasurements++;

    }


    /**
     * This will be called by the front end to do the linear algebra magic. It can run in
     * only batch solve mode (if so specified in the config file), or update the
     * information matrix decomposition incrementally via Givens rotations.
     */
    public void update() {

        // Only actually update if we're supposed to do so
        if (numSteps % updateRate == 0) {

            // Batch solve needs to take precedence (even at start)
            if (numSteps % batchSolveRate == 0 || !useIncremental) {

                if (useIncremental) {
                    System.out.println("\nUpdate "+ numSteps);
                }

                // We'll just change this to be our fastest overall method
                solve();

                // Save newly reordered factorization to disk for analysis
                if (saveRSparsity) {
                    try {
                        FileWriter fstream = new FileWriter("analysis/afterReorder.m");
                        BufferedWriter out = new BufferedWriter(fstream);

                        sparseFactor.getR().writeMatlab(out, "Rafter");
                    } catch (Exception e){
                        System.err.println("Errorz!: " + e.getMessage());
                    }
                }



            } else {

                System.out.print(".");

                // Add rows to the system
                incrementalUpdate();

                if (numSteps % solveRate == 0) {
                    // Solve via back substitution
                    solveBackSubstitution();
                }

                // Save (presumably) dense-ish R matrix to disk for analysis
                if (saveRSparsity && (numSteps+1) % batchSolveRate == 0) {
                    try {

                        FileWriter fstream = new FileWriter("analysis/beforeReorder.m");
                        BufferedWriter out = new BufferedWriter(fstream);

                        sparseFactor.getR().writeMatlab(out, "Rbefore");

                    } catch (Exception e){
                        System.err.println("Errorz!: " + e.getMessage());
                    }
                }



            }
        }


        numSteps++;
    }


    /**
     * Add the new measurements to the system via givens rotations.
     */
    private void incrementalUpdate() {

        updateNodeIndices();

        // Add each new edge to the system (if there's anything to add!)
        for (int i = edges.size()-numNewMeasurements; i < edges.size(); i++) {

            Edge anEdge = edges.get(i);

            sparseFactor.addEdgeViaGivensRotations(edges.get(i), nodeDimension);

        }

        numNewMeasurements = 0;

    }


    /**
     * Solve the SparseFactorizationSystem via back substiution and update our estimate of
     * the state vector.
     */
    private void solveBackSubstitution() {

        double[] x = getLinearizationEstimate();

        double[] deltaX = sparseFactor.solve();

        x = LinAlg.add(x, deltaX);
        updateNodesWithNewState(x);

    }


    /**
     * Find the least sqaures solution of the system. This will construct the Jacabian and
     * evaluate it at the best state vector estimate, assemble JtSigmaJ, compute the
     * residual, and find the new optimal stuff.
     */
    private void solve() {


        //WARNING: Do not use the normal gaussNewton() method with incremental updating!

        // gaussNewton();
        // fasterGaussNewton();
        // experimentalFactoringGausssNewton(new MinimumDegreeOrdering());
        experimentalFactoringGausssNewton(new SimpleDegreeOrdering());

        numNewMeasurements = 0;

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

            updateNodeLinearizationPoints();

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

                    // double[][] JatW = LinAlg.matrixAtB(edgeLin.J.get(i), edgeLin.cov);

                    // For every node associated with the edge (again)
                    for (int j = 0; j < anEdge.getNodes().size(); j++) {

                        int bIndex =  anEdge.getNodes().get(j).getIndex();

                        double[][] JatWJb = LinAlg.matrixAtB(edgeLin.J.get(i), edgeLin.J.get(j));

                        // Add the contribution of node j and i to A
                        A.plusEquals(aIndex, bIndex, JatWJb);
                    }

                    double[] JatWr = LinAlg.matrixAtB(edgeLin.J.get(i), edgeLin.residual);
                    b.plusEqualsColumnVector(aIndex, 0, JatWr);

                }

            }


            // Tikhanoff regulaization
            A = A.plus(Matrix.identity(A.getRowDimension(), A.getColumnDimension()).times(lambda));
            assert(A.isSparse());

            CholeskyDecomposition myDecomp = new CholeskyDecomposition(A);

            double[] deltaX = myDecomp.solve(b).copyAsVector();


            if (useIncremental) {
                // Hand this off to our SparseFactorizationSystem
                Matrix L = myDecomp.getL();
                Matrix rhs = myDecomp.getDecompRHS();
                sparseFactor.setR(L.transpose());
                sparseFactor.setRHS(new DenseVec(rhs.copyAsVector()));
            }

            maxChange = LinAlg.max(LinAlg.abs(deltaX));

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

        // Matrix sigmaInv = assembleInvCov();

        do {

            Matrix J = assembleJacobian();
            double[] residuals = assembleResiduals();

            // Matrix jtSig = J.transpose().times(sigmaInv);

            Matrix A = J.transpose().times(J);

            Matrix b = J.transpose().times(Matrix.columnMatrix(residuals));

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


    // /*Takes all the cov blocks from the edges and assembles the large inverse matrix*/
    // private Matrix assembleInvCov(){

    //     int curIndex = 0;

    //     /*edgeDimension is the total DOF of all edges*/
    //     Matrix cov = new Matrix(edgeDimension, edgeDimension, Matrix.SPARSE);
    //     for(Edge edge : edges){

    //         double[][] covInv = edge.getCov().inverse().copyArray();

    //         cov.set(curIndex, curIndex, covInv);

    //         curIndex += edge.getDOF();
    //     }
    //     return cov;

    // }


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


    private double[] getLinearizationEstimate() {

        int index = 0;

        double[] estimate = new double[nodeDimension];

        // Every node contributes to the state estimate
        for (Node aNode : nodes) {

            double[] nodeState = aNode.getLinearizationState();
            for (int i=0; i < aNode.getDOF(); i++) {
                estimate[index+i] = nodeState[i];
            }

            index += aNode.getDOF();

        }

        return estimate;

    }


    private void updateNodeLinearizationPoints() {
        for (Node aNode : nodes) {

            aNode.updateLinearizationPoint();

        }
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

    // public Matrix getR() {
    //     return sparseFactor.getR();
    // }


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

            updateNodeLinearizationPoints();

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

                    // double[][] JatW = LinAlg.matrixAtB(edgeLin.J.get(i), edgeLin.cov);

                    // For every node associated with the edge (again)
                    for (int j = 0; j < anEdge.getNodes().size(); j++) {

                        int bIndex =  anEdge.getNodes().get(j).getIndex();

                        double[][] JatWJb = LinAlg.matrixAtB(edgeLin.J.get(i), edgeLin.J.get(j));

                        // Add the contribution of node j and i to A
                        A.plusEquals(aIndex, bIndex, JatWJb);
                    }

                    double[] JatWr = LinAlg.matrixAtB(edgeLin.J.get(i), edgeLin.residual);
                    b.plusEqualsColumnVector(aIndex, 0, JatWr);

                }

            }


            // Tikhanoff regularization
            A = A.plus(Matrix.identity(A.getRowDimension(), A.getColumnDimension()).times(lambda));
            assert(A.isSparse());


            Matrix SA = experiemenalMakeSymbolicA();

            int saPerm[] = ordering.getPermutation(SA);

            int perm[] = new int[nodeDimension];
            int pos = 0;


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

                for (int i = 0; i < PAP.getRowDimension(); i++) {
                    if (PAP.get(i,i) == 0) {
                        PAP.set(i,i, acc / count);
                    }

                    assert(PAP.get(i,i) > 0); //System.out.printf("%10d %15f\n", i, PAP.get(i,i));
                }
            }

            CholeskyDecomposition myDecomp = new CholeskyDecomposition(PAP);

            Matrix deltaX = myDecomp.solve(PB);

            if (useIncremental) {
                // Hand this off to our SparseFactorizationSystem
                Matrix L = myDecomp.getL();
                Matrix rhs = myDecomp.getDecompRHS();
                sparseFactor.setR(L.transpose());
                sparseFactor.setRHS(new DenseVec(rhs.copyAsVector()));
                sparseFactor.setVarReordering(perm);
            }


            deltaX.inversePermuteRows(perm);


            double[] deltaXVec = deltaX.copyAsVector();

            maxChange = LinAlg.max(LinAlg.abs(deltaXVec));


            x = LinAlg.add(x, deltaXVec);
            updateNodesWithNewState(x);

            numIter++;


        } while (numIter < maxIter && maxChange > epsilon);

        return x;

    }


}
