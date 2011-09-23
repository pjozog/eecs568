package team;

import javax.swing.*; // For JFrame
import java.awt.*; // For BorderLayout
import java.util.*;

import april.vis.*; // For VisCanvas etc
import april.util.*; // For Parameter GUI
import april.jmat.*;



public class Radar
{
    public static void main(String []args){

	JFrame jf = new JFrame("VisDemo");
        jf.setSize(640,480);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        final VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);

	ParameterGUI pg = new ParameterGUI();
        pg.addDoubleSlider("sigRange", "Range Sigma", 0.0001, 100, 50.0);
        pg.addDoubleSlider("sigTheta", "Theta Sigma", 0.0001, 0.5, 0.15);
        pg.addDoubleSlider("sigCross","Range/Theta Covariance",    -10, 10, 0.0);
	pg.addDoubleSlider("meanRange", "Range Mean (m)",   0, 1000, 100);
	pg.addDoubleSlider("meanTheta", "Theta Mean (degrees)",   -180, 180, 0);
      
        jf.setLayout(new BorderLayout());
        jf.add(vc,BorderLayout.CENTER);
        jf.add(pg,BorderLayout.SOUTH);


        jf.setVisible(true);
	
        pg.addListener(new ParameterListener() {
	    public void parameterChanged(ParameterGUI pg, String name)
	    { 
		   
		if (name.equals("sigRange") || 
		    name.equals("sigTheta") ||
		    name.equals("sigCross") ||  
		    name.equals("meanRange") ||
		    name.equals("meanTheta")) {

		    double rMean     = 100;
		    double thetaMean = 0;
	
		    //Starting with r, ending with theta
		    double mean []   = new double[]{pg.gd("meanRange"), pg.gd("meanTheta")};
		    double cov[][]   = new double[2][2];

		    cov[0][0] = pg.gd("sigRange");
		    cov[0][1] = pg.gd("sigCross");
		    cov[1][0] = pg.gd("sigCross");
		    cov[1][1] = pg.gd("sigTheta");

		    MultiGaussian gauss = new MultiGaussian(cov, mean);

		    int numObservations = 500;

		    ArrayList<double[]> points = new ArrayList<double[]>();

		    Random rand = new Random();
		    for(int i = 0; i < numObservations; i++){

			double sample [] = gauss.sample(rand);

			if(sample.length != 2){
			    System.out.println("Expecting two values, got " + sample.length);
			}

			//0 is r, 1 is theta
			double r = sample[0];
			double theta = sample[1];
       
			double x = r * Math.cos(theta);
			double y = r * Math.sin(theta);
			double pt[] = new double[]{x, y};
			points.add(pt);

		    }

		    int sigma                   = 3;
		    int chi2                    = sigma * sigma;
		    MultiGaussian gaussXY       = new MultiGaussian(points);
		    ArrayList<double[]> contour = gaussXY.getContour(chi2);

		    points.addAll(contour);

		    double [][]jac = new double[2][2];
	
		    jac[0][0] = Math.cos(thetaMean);
		    jac[1][0] = Math.sin(thetaMean);
		    jac[0][1] = -rMean * Math.sin(thetaMean);
		    jac[1][1] = rMean * Math.cos(thetaMean);

		    double [][] jacT        = LinAlg.copy(LinAlg.transpose(jac));
		    double [][] projCovXY   = LinAlg.copy(LinAlg.matrixAB(jac, LinAlg.matrixAB(cov, jacT)));
		    double [] projMeanXY    = LinAlg.copy(LinAlg.matrixAB(jac, gauss.getMean()));
		    MultiGaussian projGauss = new MultiGaussian(projCovXY, projMeanXY);

		    ArrayList<double[]> projContour = projGauss.getContour(chi2);

		    points.addAll(projContour);
		
		    VisVertexData vd = new VisVertexData(points);
		    VisColorData vcd = new VisColorData();

		    for(int i = 0 ; i < numObservations; i++){
			int col = 0xff0000ff;
			vcd.add(col);
		    }
		    for(int i = 0; i < contour.size(); i++){
			int col = 0xffff0000;
			vcd.add(col);
		    }
		    for(int i = 0; i < projContour.size(); i++){
			int col = 0xff00ffff;
			vcd.add(col);
		    }

		    VisWorld.Buffer vb = vw.getBuffer("cloud");
		    vb.addBack(new VisPoints(vd, vcd, 4));
		    vb.swap();

		}
	    }
	});

	// Drawing code
	VisGrid vg = new VisGrid(new Color(.5f,.5f,.5f),
				 new Color(1.0f,1.0f,1.0f,0.0f));
	    
	VisWorld.Buffer vb = vw.getBuffer("grid");
	vb.setDrawOrder(-100);
	vb.addBack(new VisDepthTest(false,vg));
	vb.swap();

    }

}
