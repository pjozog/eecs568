package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.*;

import team.common.*;

public class SparseFactorizationSystem {

    private Matrix R;
    private DenseVec rhs;

    // Variable reordering
    private Permutation varReordering = null;

    private double EPSILON = 1e-20;

    private boolean verbose = false;
    private boolean patternVerbose = false;
    private boolean valuesVerbose = false;

    public SparseFactorizationSystem() {

        R = new Matrix(0, 0, Matrix.SPARSE);
        rhs = new DenseVec(0);

    }

    public SparseFactorizationSystem(int numRows, int numCols) {

        R = new Matrix(numRows, numCols, Matrix.SPARSE);
        rhs = new DenseVec(numRows);

    }

    public Matrix getR() {
        return R;
    }

    public DenseVec getRHS() {
        return rhs;
    }

    public Permutation getVarReordering() {
        return varReordering;
    }

    public void setR(Matrix newR) {

        if (patternVerbose) {
            System.out.println("R updated with new dimensions "+newR.getRowDimension()+ " "+newR.getColumnDimension());
            LinAlg.printPattern(newR.copyArray());
        }

        R = newR;

    }

    public void setRHS(DenseVec newRHS) {

        if (patternVerbose) {
            System.out.println("RHS updated with new dimension " + newRHS.size());
        }

        rhs = newRHS;
    }

    public void setVarReordering(int[] newPerm) {

        // I don't think it matters that this isn't deep
        varReordering = new Permutation(newPerm);

    }


    /**
     * Add an entire edge to the sparse system. This almost belongs in the BackEnd instead
     * of this file because it just assembles sparse rows from the edge's linearization and
     * passes them off to addRowViaGivensRotation(). It lives here now because the BackEnd
     * interface is getting huge.

     * This also takes care of the problem with column sizing mentioned in the BackEnd. As
     * long as newColDimension is >= what is truly needed, everything will work swimmingly.
     *
     * @param the new edge to add
     * @param the column size that the sparse system should have after this addition
     * (over-estimate)
     *
     * @return number of givens rotations performed
     */
    public int addEdgeViaGivensRotations(Edge anEdge, int newColDimension) {


        if (verbose) {
            System.out.println("ADDING AN EDGE TO THE SYSTEM with current size:");
            System.out.println("\tNew R size "+R.getRowDimension()+" "+R.getColumnDimension());
            System.out.println("\tNew rhs length "+rhs.size());
        }


        Linearization edgeLin = anEdge.getLinearization();

        // Array of associated residuals
        double[] newResiduals = edgeLin.residual;

        List<Node> nodes = anEdge.getNodes();

        int numRot = 0;

        // Construct a CSRVec for every row in the edge and add it to the system
        for (int i = 0; i < anEdge.getDOF(); i++) {


            CSRVec oneRow = new CSRVec(newColDimension);
            int newColSize = getNumCols();

            // Add the parts of each node's jacobian to the new row
            for (int k = 0; k < nodes.size(); k++) {

                Node aNode = nodes.get(k);

                int colStart = aNode.getIndex();

                if (varReordering != null) {
                    if (colStart < varReordering.perm.length) {
                        colStart = varReordering.invperm[colStart];
                    } else {
                        colStart = varReordering.perm.length;
                    }
                }

                double[][] nodeJacob = edgeLin.J.get(k);

                for (int j = 0; j < aNode.getDOF(); j++) {
                    oneRow.set(colStart + j, nodeJacob[i][j]);
                }

                // We need the vector to be exactly the right size for this edge when we
                // add it to R
                if (newColSize < colStart+aNode.getDOF()) {
                    newColSize = colStart+aNode.getDOF();
                }
            }

            // Update our variable reordering to include the newly added row
            if (varReordering != null && varReordering.perm.length < newColSize) {

                int[] currPermute = varReordering.perm;
                int[] newPermute = new int[newColSize];

                for (int v = 0; v < newColSize; v++) {

                    if (v < currPermute.length) {
                        newPermute[v] = currPermute[v];
                    } else {
                        newPermute[v] = v;
                    }

                }

                varReordering = new Permutation(newPermute);

            }

            // Make the row the correct size...necessary for dangerous sparse row
            // replacement call below.
            oneRow.resize(newColSize);
            numRot += addRowViaGivensRotation(oneRow, newResiduals[i]);
        }

        if (valuesVerbose) {
            System.out.println("R");
            LinAlg.print(R.copyArray());
            System.out.println("\nrhs");
            LinAlg.print(rhs.copyArray());
        }

        return numRot;

    }


    /**
     * Adds a row to the system and then applies givens rotations to maintain our upper
     * triangular shape that is our factorization.
     *
     * @return number of givens rotations performed
     */
    private int addRowViaGivensRotation(CSRVec newRow, double newResidual) {

        if (verbose) {
            System.out.println("addRowViaGivensRotation");
        }

        assert(R.isSparse());

        // Extend the dimensions of the system and do initial placement
        addRowToSystem(newRow, newResidual);

        // This represents the last row and non-zero columns in the now modified (and
        // non-upper-triangular) R
        int rowIndex = getNumRows() - 1;
        CSRVec lastRow = (CSRVec)R.getRow(rowIndex);
        int colIndex = lastRow.first();

        if (patternVerbose) {
            System.out.println("\tSTART colindex "+ colIndex+" nnz "+lastRow.getNz());
            LinAlg.printPattern(R.copyArray());
        }


        // Start applying givens rotations until we again arrive at a upper triangular
        // system. This shouldn't take long if we do variable reordering ocassionally.

        int numRotations = 0;

        // Only perfrom this in elements before the diagonal
        while ((colIndex >=0) && (colIndex < rowIndex)) {

            givensRotationForElement(rowIndex, colIndex);

            lastRow = (CSRVec)R.getRow(rowIndex);
            colIndex = lastRow.first();

            if (patternVerbose) {
                System.out.println("\t iter colindex "+ colIndex+" nnz "+lastRow.getNz());
                LinAlg.printPattern(R.copyArray());
            }

            numRotations++;

        }

        if (verbose) {
            System.out.println("\t* New sytem sizes R: "+R.getRowDimension()+" "+R.getColumnDimension()+"  rhs: "+rhs.size());
        }


        // It's possible that the new row wasn't associated with a new node. The row could
        // now totally be zeros. In that case, we remove it and its assocated residual.
        if (R.getRow(rowIndex).getNz() == 0) {

            int originalNumRows = getNumRows();
            R.resize(originalNumRows - 1, getNumCols());
            rhs.resize(originalNumRows-1);

        }

        return numRotations;

    }


    /**
     * Applies a givens rotation to one element in the system. It changes both R and rhs
     * by the same rotation.
     */
    private void givensRotationForElement(int row, int col) {

        assert((row >= 0) && (row < getNumRows()) && (col >=0 ) && (col < getNumCols()));
        assert(col < row);

        if (verbose) {
            System.out.println("\tgivensRotationForElement row "+row+" col "+ col);
        }

        // Get the two rows that will be changed
        CSRVec topRow = (CSRVec)R.getRow(col);
        CSRVec botRow = (CSRVec)R.getRow(row);

        // These two elements are the only two that matter to help us choose the rotation
        // angle. Golub & Van Loan section 5.1.8.
        double a = topRow.get(col);
        double b = botRow.get(col);

        //--------
        // Compute rotations
        //--------

        double c, s;

        // Algorithm 5.1.3 in Matrix Computations by Golub and Van Loan
        if (b == 0) {
            c = 1.0;
            s = 0.0;
        } else if (Math.abs(b) > Math.abs(a)) {
            double tau = -a/b;
            s = 1.0/Math.sqrt(1+tau*tau);
            c = s*tau;
        } else {
            double tau = -b/a;
            c = 1.0/Math.sqrt(1+tau*tau);
            s = c*tau;
        }


        //--------
        // Apply the Givens rotation to R
        //--------

        CSRVec newTopRow = new CSRVec(topRow.length, topRow.nz + botRow.nz);
        CSRVec newBotRow = new CSRVec(topRow.length, topRow.nz + botRow.nz);

        // From section 5.1.9 in Matrix Computations by Golub and Van Loan
        for (int j = col; j < topRow.length; j++) {

            double tauTop = topRow.get(j);
            double tauBot = botRow.get(j);

            newTopRow.set(j, c*tauTop - s*tauBot);
            if (j != col) {
                newBotRow.set(j, s*tauTop + c*tauBot);
            }
        }

        // INTERESTING: The algoritm below is actually slower than what I have above. weird. I think the
        // variable reordering takes care of a lot of things.

        // Slight optimization from april.jmat. I wish I knew they had done rotations
        // already!
        // int topRowIndex = 0;
        // int botRowIndex = 0;

        // while (topRowIndex < topRow.nz || botRowIndex < botRow.nz) {

        //     int thisIndex = Integer.MAX_VALUE;
        //     if (topRowIndex < topRow.nz)
        //         thisIndex = topRow.indices[topRowIndex];
        //     if (botRowIndex < botRow.nz)
        //         thisIndex = Math.min(thisIndex, botRow.indices[botRowIndex]);

        //     assert(thisIndex != Integer.MAX_VALUE);

        //     double tauTop = 0;
        //     double tauBot = 0;
        //     if (topRowIndex < topRow.nz && topRow.indices[topRowIndex] == thisIndex)
        //         tauTop = topRow.values[topRowIndex++];
        //     if (botRowIndex < botRow.nz && botRow.indices[botRowIndex] == thisIndex)
        //         tauBot = botRow.values[botRowIndex++];

        //     double x0r, y0r;
        //     x0r = c*tauTop - s*tauBot;
        //     y0r = s*tauTop + c*tauBot;

        //     if (x0r != 0) {
        //         newTopRow.set(thisIndex, x0r);
        //     }

        //     if (y0r != 0 && thisIndex > col) {
        //         newBotRow.set(thisIndex, y0r);
        //     }
        // }

        // Replace the rows in R with the new rows
        R.setRow(col, newTopRow);
        R.setRow(row, newBotRow);


        //--------
        // Apply same rotation to rhs
        //--------

        double r1 = rhs.get(col);
        double r2 = rhs.get(row);
        rhs.set(col, c*r1 - s*r2);
        rhs.set(row, s*r1 + c*r2);

    }


    /**
     * Actually add the row into R and the new residual component into rhs
     */
    private void addRowToSystem(CSRVec newRow, double newResidual) {

        if (verbose) {
            System.out.println("addRowToSystem");
        }

        int newColSize = Math.max(newRow.length, getNumCols());
        int newRowSize = getNumRows() + 1;

        R.resize(newRowSize, newColSize);

        // Using with extreme caution!!
        // To add the row to R, the sizes need to match perfectly.
        assert(newRow.length == getNumCols());
        R.setRow(newRowSize - 1, newRow);

        rhs.resize(newRowSize);
        rhs.set(newRowSize - 1, newResidual);

    }


    /**
     * Solve system via back substitution. This is a much faster version (borrowed from
     * the newly discovered april.jmat.Givens that exploits the vector sparsity.
     *
     * @return deltaX
     */
    public double[] solve() {

        assert(getNumRows() == getNumCols());

        int numCols = getNumCols();

        DenseVec result = new DenseVec(numCols);

        for (int i = numCols-1; i >= 0; i--) {

            CSRVec theRow = (CSRVec)R.getRow(i);
            double diag = theRow.get(i);
            double acc = theRow.dotProduct(result);

            result.set(i, (rhs.get(i) - acc)/diag);
        }


        if (varReordering != null) {

            result = (DenseVec)result.copyInversePermuteColumns(varReordering);

        }

        return result.getDoubles();
    }


    private int getNumRows() {
        return R.getRowDimension();
    }

    private int getNumCols() {
        return R.getColumnDimension();
    }
}
