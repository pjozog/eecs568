package team.slam;

import java.util.List;
import java.util.ArrayList;
import april.jmat.Matrix;

public class Linearization {

    // TODO: Make these private
    public List<double[][]> J = new ArrayList<double[][]>();
    public double[] residual;

    // This will be the inverse of the covariance matrix
    public double[][] cov;

}
