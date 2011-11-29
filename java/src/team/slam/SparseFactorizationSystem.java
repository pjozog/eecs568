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


    public void addRowViaGivensRotation(CSRVec newRow, double newResidual) {
        
    }
}
