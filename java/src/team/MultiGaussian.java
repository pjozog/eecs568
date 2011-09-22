package team;

import april.jmat.*;

import java.util.*;

/** Represents a multi-variate Gaussian distribution. This is a
 * template for EECS568; you should fill in the methods below.
 **/
public class MultiGaussian
{
    /** The mean **/
    double u[];

    /** The covariance **/
    double P[][];

    /** Create a new MultiGaussian object with covariance P and mean u.
        (This should be trivial)
    **/
    public MultiGaussian(double P[][], double u[])
    {
        // You shouldn't need to modify this.
        this.P = LinAlg.copy(P);
        this.u = LinAlg.copy(u);
    }

    /** Create a new MultiGaussian object that estimates the covariance and mean
        from the provided samples. This should implement your algorithm from A. **/
    public MultiGaussian(ArrayList<double[]> samples)
    {
	//Compute sample mean
	//Sum over all values of samples
	int sampleLength = samples.get(0).length;
	int N = samples.size();
	double[] sum = new double[sampleLength];

	for (int i=0; i<N; i++) 
	    sum = LinAlg.add(samples.get(i), sum);

	this.u = LinAlg.scale(sum, 1.0/N);

	//Compute sample covariance
	//cov is 1.0/N (X-mean)(X-mean)'
	//      =1.0/N XPrime*XPrime'
	double[][] XPrime = new double[sampleLength][samples.size()];

	for (int i = 0; i < N; i++) {

	    double[] temp = LinAlg.subtract(samples.get(i), this.u);
	    for (int j = 0; j < temp.length; j++) {
		XPrime[j][i] = temp[j];
	    }

	}

	double[][] XPrimeT = LinAlg.copy(LinAlg.transpose(XPrime));

	this.P = LinAlg.scale(LinAlg.matrixAB(XPrime, XPrimeT), 1.0/N);
	
    }

    /** Return the covariance associated with this object. (Trivial). **/
    public double[][] getCovariance()
    {
        // You shouldn't need to modify this.
        return P;
    }

    /** Return the mean associated with this object. (Trivial). **/
    public double[] getMean()
    {
        // You shouldn't need to modify this.
        return u;
    }

    /** Draw a random sample from this distribution. This should implement your
        method from part C.
    **/
    public double[] sample(Random r)
    {

	Matrix L;
	CholeskyDecomposition chol = new CholeskyDecomposition(new Matrix(this.P));
	L = chol.getL();

	//The mean
	double[] b = this.u;

	//The N(0,1) samples
        double x[] = new double[u.length];
        for (int i = 0; i < x.length; i++) {
            x[i] = r.nextGaussian();
        }

	//We need to multiply x by "A", which is just L from above, and add b
	// ie: y = L*x + b;
	double[] y;
	y = LinAlg.add(LinAlg.matrixAB(L.copyArray(), x), b);

        return y;
    }

    /** Given an observation from the distribution, compute the chi^2 value. This
        is given by (x-u)'inv(M)(x-u)
    **/
    public double chi2(double[] x)
    {
	double[] xMinusU      = LinAlg.subtract(x, this.u);
	double[][] inverseCov = LinAlg.inverse(this.P);
	if (inverseCov == null)
	    return -1;

	double c2 = LinAlg.dotProduct(LinAlg.matrixAB(xMinusU, inverseCov), xMinusU);

	return c2;
    }

    /** Compute a set of points that, when plotted as a curve, would trace out an
        iso-probability contour corresponding to the specified chi^2 value. Generate
        points at one-degree spacings using your method from part D.
    **/
    public ArrayList<double[]> getContour(double chi2)
    {
        //  XXX Write me
        return null;
    }

    /** Compare sample mean and covariance to ground truth mean and
	covariance
    **/
    public static void testMultiGaussian()
    {
	//Our "ground truth"" distribution: correlated gaussian
	double[][] P = new double[2][2];
	P[0][0] = 1; P[0][1] = .5;
	P[1][1] = 1; P[1][0] = .5;

	double[] u = new double[2];
	u[0] = 0;
	u[1] = 0;

	//Instantiate MVG with true covariance and mean
	MultiGaussian trueMultiGaussian = new MultiGaussian(P, u);

	Random rand = new Random();
	ArrayList<double[]> samples = new ArrayList<double[]>();
	for (int i = 0; i < 100000; i++)
	    samples.add(trueMultiGaussian.sample(rand));

	//Instantiate other MVG with samples drawn from MVG above.  It
	//should have the same mean and covariance...
	MultiGaussian testMultiGaussian = new MultiGaussian(samples);
	
	System.out.println("Sample mean (should be 0 vector):");
	double[] mean = testMultiGaussian.getMean();
	ArrayUtil.print1dArray(mean);
	System.out.println("");

	System.out.printf("Sample covariance (should be [%.1f %.1f; %.1f %.1f]):\n", 
			  P[0][0], P[0][1], P[1][0], P[1][1]);
	double[][] covariance = testMultiGaussian.getCovariance();
	ArrayUtil.print2dArray(covariance);
	System.out.println("");
	
	double[] sample0 = new double[]{0, 0};
	double[] sample1 = new double[]{.5, .5};

	//Compare chi2 to what we determined in MATLAB (trivial case)
	System.out.println("chi2 values for X = [0; 0] (should be 0)");
	System.out.print("Exact MVG:  ");
	System.out.println(trueMultiGaussian.chi2(sample0));
	System.out.print("Sample MVG: ");
	System.out.println(testMultiGaussian.chi2(sample0));
	System.out.println("");

	//Compare chi2 to what we determined in MATLAB (more arbitrary case)
	System.out.println("chi2 values for X = [.5; .5] (should be 1/3):");
	System.out.print("True MVG:   ");
	System.out.println(trueMultiGaussian.chi2(sample1));
	System.out.print("Sample MVG: ");
	System.out.println(testMultiGaussian.chi2(sample1));

    }

    public static void main(String args[])
    {
        // Insert your test code here.
	testMultiGaussian();

    }
}
