package team.scan;

import java.util.*;
import java.awt.*;

import april.vis.*;
import team.*;

public class AggloLineFit {

    ArrayList<double[]> points;
    ArrayList<Line> lines;
    VisWorld.Buffer lineBuff;

    public static final int MAXITER = 1000;
    public static Random rand = new Random();

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


        double numIter      = Task2.getSteps();
        double MSEThreshold = Task2.getThreshold();

        //Go forth!
        for(int step = 0; step < numIter; step++){
            //Need to keep track of some stuff...
            double lowestMSE        = Double.POSITIVE_INFINITY;
            int    lowestMSEIndex1  = -1;
            int    lowestMSEIndex2  = -1;
            Line   mergedLine       = null;
            Line   lowestMergedLine = null;

            //for each pair of adjacent lines i, i+1
            for (int i = 0; i < lines.size()-1; i++) {

                Line currentLine = this.lines.get(i);
                Line nextLine = this.lines.get(i+1);

                mergedLine = Line.mergeLines(currentLine, nextLine);

                // The line will be null if the join exceeded some distance threshold.
                if (mergedLine != null) {
                    double MSE = mergedLine.computeMSE();

                    //Find the minimum error and lines with minimum error
                    if (MSE < lowestMSE) {
                        lowestMSE = MSE;
                        lowestMSEIndex1 = i;
                        lowestMSEIndex2 = i+1;
                        lowestMergedLine = mergedLine;
                    }
                }


            }

            //if minimum error > thresh, then we're done
            if (lowestMSE > MSEThreshold) {

                //Draw the shits, for debugging
                this.lineBuff.clear();

                for (Line l : Line.removeTwoPointLines(this.lines)) {

                    float r = rand.nextFloat();
                    float g = rand.nextFloat();
                    float b = rand.nextFloat();

                    this.lineBuff.addBack(new VisLines(new VisVertexData(l.getPointsForDisplay()),
                                                       new VisConstantColor(Color.red),
                                                       2, VisLines.TYPE.LINES));
                }

                lineBuff.swap();

                double THIS_IS_FOR_BREAKPOINT = 1337;

                return this.lines;
            }

            else {
                //Merge lines with minimum error
                this.lines.set(lowestMSEIndex1, lowestMergedLine);
                this.lines.remove(lowestMSEIndex2);
                assert(lowestMSEIndex2 == lowestMSEIndex1 + 1);
            }

            //Repeat


        }

        return this.lines;
    }

    /**
     * Create n-1 lines for each pair of adjacent points
     */
    private void init() {
        for (int i = 0; i < points.size()-1; i++) {
            Line newLine = Line.initialLine(points.subList(i, i+2));
            if (newLine != null) {
                lines.add(newLine);
            }
        }
    }

}
