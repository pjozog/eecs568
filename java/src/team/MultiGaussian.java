package team;

import javax.swing.*; // For JFrame
import java.awt.*; // For BorderLayout
import java.util.*;

import april.vis.*; // For VisCanvas etc
import april.util.*; // For Parameter GUI
import april.jmat.*;


/** Represents a multi-variate Gaussian distribution. This is a
 * template for EECS568; you should fill in the methods below.
 **/
public class MultiGaussian
{

    private final double DEG_TO_RAD = Math.PI/180.0;
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
	double[] xMinusU      = LinAlg.subtract(x, getMean());
	double[][] inverseCov = LinAlg.inverse(getCovariance());
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
	ArrayList<double[]> result = new ArrayList<double[]>();
	
	double[][] invCov = LinAlg.inverse(getCovariance());

	if(invCov == null){

	    return null;
	}
	
	double a = invCov[0][0];
	double b = invCov[1][1];
	
	//same as [1][0] hopefully
	double c = invCov[0][1];
	
	for(int theta = 0; theta < 359; theta++){
	    double thetaRad = theta * DEG_TO_RAD;
	    double alpha = Math.sqrt(chi2/
				(a * Math.pow(Math.cos(thetaRad), 2) + 
				 b * Math.pow(Math.sin(thetaRad), 2) + 
				 2 * c * Math.cos(thetaRad) * Math.sin(thetaRad))
				);
	
	    double [] mu = getMean();
	    double [] box = new double [2];
	    
	    box[0] = alpha * Math.cos(thetaRad);
	    box[1] = alpha * Math.sin(thetaRad);
	    
	    double [] p  = LinAlg.add(mu, box);
	    result.add(p);
	}
	
	

      
	
        return result;
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
	
        // Initialization
        JFrame jf = new JFrame("VisDemo");
        jf.setSize(640,480);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        final VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);



	ParameterGUI pg = new ParameterGUI();
        pg.addDoubleSlider("sig1", "Sigma 1-1", 0.0001, 10, 3.0);
        pg.addDoubleSlider("sig2", "Sigma 2-2", 0.0001, 10, 2.0);
        pg.addDoubleSlider("sig12","Sigma 1-2",    -10, 10, 1.0);

	pg.addDoubleSlider("meanx", "Mean X",   -10, 10, 0);
	pg.addDoubleSlider("meany", "Mean Y",   -10, 10, 0);

	pg.addDoubleSlider("stddev",  "Std Dev",    0, 5, 0); 

	pg.addString("percent", "Percent of Points in Contour", "");
      
        jf.setLayout(new BorderLayout());
        jf.add(vc,BorderLayout.CENTER);
        jf.add(pg,BorderLayout.SOUTH);


        jf.setVisible(true);

        // Drawing code
        {
            VisGrid vg = new VisGrid(new Color(.5f,.5f,.5f),
                                     new Color(1.0f,1.0f,1.0f,0.0f));

            VisWorld.Buffer vb = vw.getBuffer("grid");
            vb.setDrawOrder(-100);
            vb.addBack(new VisDepthTest(false,vg));
            vb.swap();
        }


        pg.addListener(new ParameterListener(){
                public void parameterChanged(ParameterGUI pg, String name)
                { 
		   
                    if (name.equals("sig1") || name.equals("sig2") 
			|| name.equals("sig12") ||  name.equals("meanx") 
			|| name.equals("meany") || name.equals("stddev")) {
		
			ArrayList<double[]> points  = new ArrayList<double[]>();
                        
			
			Random rand = new Random();
			
                        		
			double [] means = new double[]{pg.gd("meanx"), pg.gd("meany")};
			double [][] cov = new double[2][2];
			
			cov[0][0] = pg.gd("sig1"); 
			cov[0][1] = pg.gd("sig12"); 
			cov[1][0] = pg.gd("sig12"); 
			cov[1][1] = pg.gd("sig2"); 
		


			MultiGaussian gauss = new MultiGaussian(cov, means);
			
			int randomPoints = 1000;
                        for (int i = 0; i < randomPoints; i++) {
			    
                            double pt[] = gauss.sample(rand);

			    points.add(pt);
			   
                        }

			double chi2 = Math.pow(pg.gd("stddev"), 2);;
			ArrayList<double[]> contourPoints = gauss.getContour(chi2);
			if(contourPoints == null){
			    return;   
			}
			points.addAll(contourPoints);
		       

			pg.ss("percent", "" + rand.nextInt());


                        VisVertexData vd = new VisVertexData(points);

                        VisColorData vcd = new VisColorData();

			
			for(int i = 0; i < points.size() ; i++){
			    int col = 0;
			    if(i < 1000){
				col = 0xff0000ff;
			    }
			    else{
				col = 0xffff0000;
			    }
			    vcd.add(col);

			}

                        VisWorld.Buffer vb = vw.getBuffer("cloud");
                        vb.addBack(new VisPoints(vd, vcd, 4));
                        // vb.addBack(new VisLines(vd,vcd, 4, VisLines.TYPE.LINES));
                        vb.swap();
                    }
                }
            });


	
	//Ping the gui to show up without slider movement.
	pg.notifyListeners("stddev");
	
    }
}
