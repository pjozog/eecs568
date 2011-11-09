package team.scan;

import java.util.*;
import java.awt.*;

import april.vis.*;

public class AggloLineFit {

    ArrayList<double[]> points;
    ArrayList<Line> lines;
    VisWorld.Buffer lineBuff;

    public static final int MAXITER = 1000;
    
    public AggloLineFit(ArrayList<double[]> points, VisWorld.Buffer lineBuff) {

        this.points = new ArrayList<double[]>();
        this.lines  = new ArrayList<Line>();
	this.lineBuff = lineBuff;

        for (double[] point : points) {
            this.points.add(point);
        }

    }

    public ArrayList<Line> getLines() {

        //Populate this.lines
        this.init();

        double numIter      = 0;
        double MSEThreshold = 100;

        //Go forth!
        while (true) {

            //Need to keep track of some stuff...
            double lowestMSE = Double.POSITIVE_INFINITY;
            int   lowestMSEIndex1 = -1;
            int   lowestMSEIndex2 = -1;
            Line mergedLine = null;

            //for each pair of adjacent lines i, i+1
            for (int i = 0; i < lines.size()-1; i++) {

                Line currentLine = this.lines.get(i);
                Line nextLine = this.lines.get(i+1);

                mergedLine = new Line(currentLine, nextLine);
                double MSE = mergedLine.computeMSE();

                //Find the minimum error and lines with minimum error
                if (MSE < lowestMSE) {
                    lowestMSE = MSE;
                    lowestMSEIndex1 = i;
                    lowestMSEIndex2 = i+1;
                }

            }

            //if minimum error > thresh, then we're done
            if (lowestMSE > MSEThreshold) {
                return this.lines;
            } 
            
            else {
                //Merge lines with minimum error
                this.lines.set(lowestMSEIndex1, mergedLine);
                this.lines.remove(lowestMSEIndex2);
                assert(lowestMSEIndex1 == lowestMSEIndex2 + 1);
            }

            //Repeat

	    //Draw the shits, for debugging
	    this.lineBuff.clear();
	    for (Line l : this.lines) {
		this.lineBuff.addBack(new VisLines(new VisVertexData(l.getPointsForDisplay()),
						   new VisConstantColor(Color.blue),
						   2, VisLines.TYPE.LINES));
	    }
	    lineBuff.swap();
            double fml = 1337;
        }

    }

    /**
     * Create n-1 lines for each pair of adjacent points
     */
    private void init() {
        for (int i = 0; i < points.size()-1; i++) {
            lines.add(new Line(points.subList(i, i+2)));
        }
    }

}
