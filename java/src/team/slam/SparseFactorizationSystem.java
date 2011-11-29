package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;
import april.jmat.LinAlg;
import april.jmat.DenseVec;
// import april.jmat.CholeskyDecomposition;

import team.common.*;

public class SparseFactorizationSystem {

    private Matrix R;
    private DenseVec rhs;


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

    public void addEdgeViaGivensRotations(Edge anEdge, int newColDimension) {

        Linearization edgeLin = anEdge.getLinearization();

        // Array of associated residuals
        double[] newResiduals = edgeLin.residual;

        List<Nodes> nodes = anEdge.getNodes();

        // Construct a CSRVec for every row in the edge and add it to the system
        for (int i = 0; i < anEdge.getDOF(); i++) {

            CSRVec oneRow = new CSRVec(newColDimension);

            // Add the parts of each node's jacobian to the new row
            for (k = 0; k < nodes.size(); k++) {

                int colStart = aNode.getIndex();

                double[][] nodeJacob = edgeLin.J.get(k);

                for (int j = 0; j < aNode.getDOF(); j++) {
                    oneRow.set(colStart + j, nodeJacob[i][j])
                }
            }

            addRowViaGivensRotation(oneRow, newResiduals[i]);
        }
    }

    public void addRowViaGivensRotation(CSRVec newRow, double newResidual) {

        // Extend the dimensions of the system and do initial placement
        addRowToSystem(newRow, newResidual);

        // Start applying givens rotations until we again arrive at a upper triangular
        // system. This shouldn't take long if we do variable reordering ocassionally.
        int rowIndex = getNumRows();
        int colIndex = newRow.


            }

    private void givensRotation(int row, int col) {

    }

    private void addRowToSystem(CSRVec newRow, double newResidual) {

    }

    private int getNumRows() {
        return R.getRowDimension();
    }
}
