package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.*;

import team.common.*;

public class SparseFactorizationSystem {

    private Matrix R;
    private DenseVec rhs;

    private double EPSILON = 1e-8;

    public SparseFactorizationSystem() {

        R = new Matrix(1, 1, Matrix.SPARSE);
        rhs = new DenseVec(1);

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
                if (newColSize < colStart+aNode.getDOF()+1) {
                    newColSize = colStart+aNode.getDOF()+1;
                }
            }


            oneRow.resize(newColSize);
            addRowViaGivensRotation(oneRow, newResiduals[i]);
        }
    }

    /**
     * Solve system via back substitution.
     *
     * @return deltaX
     */
    public double[] solve() {

        return new double[1];
    }

    /**
     * Adds a row to the system and then applies givens rotations to maintain our upper
     * triangular shape that is our factorization.
     */
    private void addRowViaGivensRotation(CSRVec newRow, double newResidual) {

        // Extend the dimensions of the system and do initial placement
        addRowToSystem(newRow, newResidual);

        // This represents the last row and non-zero columns in the now modified (and
        // non-upper-triangular) R
        int rowIndex = getNumRows() - 1;
        int colIndex = R.getRow(rowIndex).first();

        // Start applying givens rotations until we again arrive at a upper triangular
        // system. This shouldn't take long if we do variable reordering ocassionally.

        // Only perfrom this in elements before the diagonal
        while ((colIndex >=0) && (colIndex < rowIndex)) {
            givensRotationForElement(rowIndex, colIndex);
            colIndex =  R.getRow(rowIndex).first();
        }

        // It's possible that the new row wasn't associated with a new node. The row could
        // now totally be zeros. In that case, we remove it and its assocated residual.
        // TODO: In our situation, I don't think this will ever happen?
        if (R.getRow(rowIndex).getNz() == 0) {
            // This is for you Schuyler
            assert(false);
        }


    }

    private void givensRotationForElement(int row, int col) {

        assert((row >= 0) && (row < getNumRows()) && (col >=0 ) && (col < getNumCols()));
        assert(col < row);

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
            newBotRow.set(j, s*tauTop + c*tauBot);
        }

        // Remove any elements that should be zero...machine precision issues
        newTopRow.filterZeros(EPSILON);
        newBotRow.filterZeros(EPSILON);

        // The whole point of this was to make this one element zero!
        assert(newBotRow.get(col) == 0);

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

        int newColSize = Math.max(newRow.length, getNumCols());
        int newRowSize = getNumRows() + 1;

        R.resize(newRowSize, newColSize);

        // Using with extreme caution!!
        // To add the row to R, the sizes need to match perfectly.
        assert(newRow.length == getNumCols());
        R.setRow(newRowSize-1, newRow);

        rhs.resize(newRowSize);
        rhs.set(getNumRows()-1, newResidual);

    }



    private int getNumRows() {
        return R.getRowDimension();
    }

    private int getNumCols() {
        return R.getColumnDimension();
    }
}
