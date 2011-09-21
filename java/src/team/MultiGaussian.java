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

	// for (int i = 0; i < sampleLength; i++) {
	//     for (int j = 0; j<sampleLength; j++) {
	// 	System.out.print(XPrimeT[i][j] + " ");		
	//     }
	//     System.out.println();
	// }
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
        // XXX The code below is NOT correct, but demonstrates some of
        // the APIs you might use.

	Matrix L;
	CholeskyDecomposition chol = new CholeskyDecomposition(new Matrix(this.P));
	L = chol.getL();

	//The mean
	double[] b = this.u;
	//The covariance:

        double x[] = new double[u.length];
        for (int i = 0; i < x.length; i++) {
            x[i] = r.nextGaussian();
        }

	//We need to multiply x by "A", which is just L from above, and add b
	// y = L*x + b;
	double[] y;
	y = LinAlg.add(LinAlg.matrixAB(L.copyArray(), x), b);

        return y;
    }

    /** Given an observation from the distribution, compute the chi^2 value. This
        is given by (x-u)'inv(M)(x-u)
    **/
    public double chi2(double[] x)
    {
	double[] xMinusU = LinAlg.subtract(x, this.u);
	// for (int i = 0; i < xMinusU.length; i++)
	//     System.out.println(xMinusU[i]);

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

    //Compare sample mean and covariance to ground truth mean and
    //covariance
    public static void testMultiGaussian()
    {
	//Our "ground" truth distribution: correlated normal gaussian
	double[][] P = new double[2][2];
	P[0][0] = 1; P[0][1] = .5;
	P[1][1] = 1; P[1][0] = .5;

	double[] u = new double[2];
	u[0] = 0;
	u[1] = 0;

	MultiGaussian trueMultiGaussian = new MultiGaussian(P, u);

	Random rand = new Random();
	ArrayList<double[]> samples = new ArrayList<double[]>();
	for (int i = 0; i < 100000; i++)
	    samples.add(trueMultiGaussian.sample(rand));

	MultiGaussian testMultiGaussian = new MultiGaussian(samples);
	
	System.out.println("Sample mean (should be 0 vector):");
	double[] mean = testMultiGaussian.getMean();
	for (int row = 0; row<mean.length; row++) {
	    System.out.println(testMultiGaussian.getMean()[row]);
	}
	System.out.println("");

	System.out.println("Sample covariance (should be [1 .5; .5 1]):");
	double[][] covariance = testMultiGaussian.getCovariance();
	for (int row = 0; row < covariance.length; row++) {
	    for (int col = 0; col < covariance[0].length; col++) {
		System.out.print(covariance[row][col] + " ");
	    }
	    System.out.println();
	}
	System.out.println("");
	
	double[] sample0 = new double[]{0, 0};
	double[] sample1 = new double[]{.5, .5};

	System.out.println("chi2 values for [0; 0]:");
	System.out.print("Ground Truth: ");
	System.out.println(trueMultiGaussian.chi2(sample0));
	System.out.print("Test:         ");
	System.out.println(testMultiGaussian.chi2(sample0));
	System.out.println("");

	System.out.println("chi2 values for [.5; .5]:");
	System.out.print("Ground Truth: ");
	System.out.println(trueMultiGaussian.chi2(sample1));
	System.out.print("Test:         ");
	System.out.println(testMultiGaussian.chi2(sample1));

    }

    public static void main(String args[])
    {
        // Insert your test code here.
	testMultiGaussian();

    }
}
