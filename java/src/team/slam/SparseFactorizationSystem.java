package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.*;

import team.common.*;

public class SparseFactorizationSystem {

    private Matrix R;
    private DenseVec rhs;

    private double EPSILON = 1e-20;

    private boolean verbose = false;
    private boolean patternVerbose = false;
    private boolean valuesVerbose = true;

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


    /**
     * Add an entire edge to the sparse system. This almost belongs in the BackEnd instead
     * of here, because it just assembles sparse rows from the edge's linearization and
     * passes them off to addRowViaGivensRotation(). It lives here now because the BackEnd
     * is getting huge.
     *
     * @param the new edge to add
     * @param the column size that the sparse system should have after this addition
     */
    public void addEdgeViaGivensRotations(Edge anEdge, int newColDimension) {


        if (verbose) {
            System.out.println("ADDING AN EDGE TO THE SYSTEM with current size:");
            System.out.println("\tNew R size "+R.getRowDimension()+" "+R.getColumnDimension());
            System.out.println("\tNew rhs length "+rhs.size());
        }


        Linearization edgeLin = anEdge.getLinearization();

        // Array of associated residuals
        double[] newResiduals = edgeLin.residual;

        List<Node> nodes = anEdge.getNodes();

        // Construct a CSRVec for every row in the edge and add it to the system
        for (int i = 0; i < anEdge.getDOF(); i++) {


            CSRVec oneRow = new CSRVec(newColDimension);
            int newColSize = getNumCols();

            // Add the parts of each node's jacobian to the new row
            for (int k = 0; k < nodes.size(); k++) {

                Node aNode = nodes.get(k);

                int colStart = aNode.getIndex();

                double[][] nodeJacob = edgeLin.J.get(k);

                for (int j = 0; j < aNode.getDOF(); j++) {
                    oneRow.set(colStart + j, nodeJacob[i][j]);
                }

                // We need the vector to be exactly the right size for this edge when we
                // add it to R
                // TODO: Is the +1 correct here?
                if (newColSize < colStart+aNode.getDOF()) {
                    newColSize = colStart+aNode.getDOF();
                }
            }

            // FIX: Probably not necessary. There's a first() call below that sorts things.
            oneRow.performSort();

            // System.out.println("Adding a row to the sparse system");
            // LinAlg.printTranspose(oneRow.copyArray());
            oneRow.resize(newColSize);
            addRowViaGivensRotation(oneRow, newResiduals[i]);
        }

        // LinAlg.printPattern(R.copyArray());
        if (valuesVerbose) {
            System.out.println("R");
            LinAlg.print(R.copyArray());
            System.out.println("\nrhs");
            LinAlg.print(rhs.copyArray());
        }


    }


    /**
     * Adds a row to the system and then applies givens rotations to maintain our upper
     * triangular shape that is our factorization.
     */
    private void addRowViaGivensRotation(CSRVec newRow, double newResidual) {

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

        // Only perfrom this in elements before the diagonal
        while ((colIndex >=0) && (colIndex < rowIndex)) {

            givensRotationForElement(rowIndex, colIndex);

            lastRow = (CSRVec)R.getRow(rowIndex);
            colIndex = lastRow.first();

            if (patternVerbose) {
                System.out.println("\t iter colindex "+ colIndex+" nnz "+lastRow.getNz());
                LinAlg.printPattern(R.copyArray());
            }

        }

        if (verbose) {
            System.out.println("\t* New sytem sizes R: "+R.getRowDimension()+" "+R.getColumnDimension()+"  rhs: "+rhs.size());
        }


        // It's possible that the new row wasn't associated with a new node. The row could
        // now totally be zeros. In that case, we remove it and its assocated residual.
        // TODO: In our situation, I don't think this will ever happen?
        // FIX: This will definitely happen!
        if (R.getRow(rowIndex).getNz() == 0) {
            // This is for you Schuyler
            // System.out.println("I wasn't ready for this...");
            // assert(false);
            System.out.println("REMOVING ROW! Is this correct?");
            // assert(false);

            int originalNumRows = getNumRows();
            R.resize(originalNumRows - 1, getNumCols());
            rhs.resize(originalNumRows-1);

            System.out.println("\tNew R size "+R.getRowDimension()+" "+R.getColumnDimension());
            System.out.println("\tNew rhs length "+rhs.size());


        }


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

        CSRVec newTopRow = new CSRVec(topRow.length);
        CSRVec newBotRow = new CSRVec(topRow.length);

        // From section 5.1.9 in Matrix Computations by Golub and Van Loan
        for (int j = col; j < topRow.length; j++) {

            double tauTop = topRow.get(j);
            double tauBot = botRow.get(j);

            newTopRow.set(j, c*tauTop - s*tauBot);
            if (j != col) {
                newBotRow.set(j, s*tauTop + c*tauBot);
            }
        }

        // if (row == 22 && col == 20) {
        //     LinAlg.print(newBotRow.copyArray());
        // }

        // The whole point of this was to make this one element zero!
        // assert(newBotRow.get(col) == 0);
        newBotRow.set(col, 0);

        int numNzBot = newBotRow.getNz();

        // // Remove any elements that should be zero...machine precision issues
        // // newTopRow.filterZeros(EPSILON);
        // // newBotRow.filterZeros(EPSILON);

        // newTopRow.filterZeros();
        newBotRow.filterZeros();

        if (numNzBot != newBotRow.getNz()) {
            System.out.println("WARNING WARNING!!!!!!!! The machine precision filter removed an element! Sizes "+numNzBot+" " + newBotRow.getNz());
            // assert(false);
        }


        // if (row == 22 && col == 20) {
        //     LinAlg.print(newBotRow.copyArray());
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
     * Solve system via back substitution.
     *
     * @return deltaX
     */
    public double[] solve() {

        assert(getNumRows() == getNumCols());

        int numCols = getNumCols();

        DenseVec result = new DenseVec(numCols);

        for (int rowIndex = numCols-1; rowIndex >= 0; rowIndex--) {

            CSRVec theRow = (CSRVec)R.getRow(rowIndex);
            double elem = rhs.get(rowIndex);

            for (int colIndex = theRow.first() ; colIndex < theRow.length; colIndex++) {

                double v = theRow.get(colIndex);

                if (rowIndex != colIndex) {
                    elem = elem - result.get(colIndex)*v;
                }
            }

            double diag = theRow.get(rowIndex);

            if (diag == 0.0) {
                System.out.println("Matrix is not upper triangular!");
                assert(false);
            }

            result.set(rowIndex, elem/diag);

        }

        if (valuesVerbose) {
            System.out.println("Incremental X");
            LinAlg.print(result.copyArray());
        }

        return result.copyArray();
    }


    private int getNumRows() {
        return R.getRowDimension();
    }

    private int getNumCols() {
        return R.getColumnDimension();
    }
}
