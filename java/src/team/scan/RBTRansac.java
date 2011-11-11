package team.scan;

import team.*;
import april.vis.*;
import java.util.*;

public class RBTRansac {

    private List<Corner> cornersA;
    private List<Corner> cornersB;

    private List<Line> linesA;
    private List<Line> linesB;

    private double consensusThresh;

    public RBTRansac(ArrayList<double[]> pointsA, ArrayList<double[]> pointsB,
                     VisWorld.Buffer lineBuffA, VisWorld.Buffer lineBuffB,
                     int numIter, double threshold) {

        AggloLineFit lineFitterA = new AggloLineFit(pointsA, lineBuffA, numIter, threshold);
        AggloLineFit lineFitterB = new AggloLineFit(pointsB, lineBuffB, numIter, threshold);

        //get the lines
        this.linesA = lineFitterA.getLines();
        this.linesB = lineFitterB.getLines();

        //get the corners
        this.cornersA = Corner.getAllCorners(this.linesA);
        this.cornersB = Corner.getAllCorners(this.linesB);

        consensusThresh = threshold;
    }

    //Bjarne would hate us but this returns x, y, and theta describing
    //2D RBT.  Use
    private double[] doRansac() {

        //loop
        //find RBT
        //apply to all B points
        //inliers = conScore
        //if(inliers > maxIn) new best

        return null;
    }

    private double consensusScore(List<double []> listA, List<double []> listB) {

        double count = 0;
        for(double [] aPos : listA){
            for(double [] bPos : listB){
                double x1 = aPos[0];
                double y1 = aPos[1];
                double x2 = bPos[0];
                double y2 = bPos[1];
                
                double dist = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
                if(dist < consensusThresh){
                    count = count + 1;
                }
            }
        }

        //for points in a
        //for points in b
        //  compute dist from a to b, count num points inside circle
        
        return count;
    }

}