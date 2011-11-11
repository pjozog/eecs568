package team.scan;

import team.*;
import april.vis.*;
import java.util.*;
import april.jmat.*;

public class RBTRansac {

    private int numIterations = 40;

    private double consensusThresh;

    private List<Corner> cornersA;
    private List<Corner> cornersB;

    private List<Line> linesA;
    private List<Line> linesB;

    private List<double[]> pointsA;
    private List<double[]> pointsB;

    public RBTRansac(ArrayList<double[]> pointsA, ArrayList<double[]> pointsB,
                     VisWorld.Buffer lineBuffA, VisWorld.Buffer lineBuffB,
                     int numIter, double threshold, double conThreshold) {


        System.out.println("Num points " + pointsA.size() + " " + pointsB.size());
        AggloLineFit lineFitterA = new AggloLineFit(pointsA, lineBuffA, numIter, threshold);
        AggloLineFit lineFitterB = new AggloLineFit(pointsB, lineBuffB, numIter, threshold);

        //get the lines
        this.linesA = lineFitterA.getLines(false);
        this.linesB = lineFitterB.getLines(false);
        System.out.println("Num lines " + linesA.size() + " " + linesB.size());

        //get the corners
        this.cornersA = Corner.getAllCorners(this.linesA);
        this.cornersB = Corner.getAllCorners(this.linesB);

        System.out.println("Num corners " + cornersA.size() + " " + cornersB.size());
        this.pointsA = pointsA;
        this.pointsB = pointsB;

        this.consensusThresh = conThreshold;

    }

    //Bjarne would hate us but this returns x, y, and theta describing
    //2D RBT.  Use
    public ArrayList<double[]> doRansac() {

        if (cornersA.size() == 0 || cornersB.size() == 0) {
            return null;
        }

        //BestModel
        double [] bestRBT = null;
        double bestConsensus = -1.0;

        Random randGuy = new Random(0);

        for (int i = 0; i < numIterations; i++) {

            // Select two corners at random
            int indexOne = randGuy.nextInt(cornersA.size());
            int indexTwo = randGuy.nextInt(cornersB.size());

            Corner cornerOne = cornersA.get(indexOne);
            Corner cornerTwo = cornersB.get(indexTwo);

            // Compute the RBT between the two corners
            double[] theRBT = rbtFromCorners(cornerOne, cornerTwo);

            // Perform the RBT on the second scan's points
            ArrayList<double[]> newPointsB = applyTransform(theRBT, pointsB);

            // Compute the consesus score with the transformed points and the first scan's points
            double currConsesus = consensusScore(pointsA, newPointsB);

            if (currConsesus > bestConsensus) {
                bestConsensus = currConsesus;
                bestRBT = new double[] {theRBT[0], theRBT[1], theRBT[2]};
            }
        }

        return applyTransform(bestRBT, pointsB);
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

    private double[] rbtFromCorners(Corner one, Corner two) {

        double thetaOne = one.getTheta();
        double thetaTwo = two.getTheta();

        double[] xyPosOne = one.getXYPos();
        double[] xyPosTwo = two.getXYPos();


        double theta = thetaOne - thetaTwo;
        double dX = xyPosOne[0] - Math.cos(theta)*xyPosTwo[0] + Math.sin(theta)*xyPosTwo[1];
        double dY = xyPosOne[1] - Math.sin(theta)*xyPosTwo[0] - Math.cos(theta)*xyPosTwo[1];

        return new double[] {dX, dY, theta};

    }

    //Given a RBT (x,y,theta) from frame A to B, this *should* apply
    //RBT on points in frame A, and the resulting ArrayList will be
    //points in frame B
    private ArrayList<double[]> applyTransform(double[] RBT, List<double[]> points) {
        ArrayList<double[]> transformedPoints = new ArrayList<double[]>();

        for (double[] point : points) {

            //Make the point homogeneous
            double[] homoPoint   = new double[]{point[0], point[1], 0, 1.0};
            double[][] RBTMatrix = LinAlg.xytToMatrix(RBT);
            /*
            for(int i = 0; i < RBTMatrix.length; i++){
                for(int j = 0; j < RBTMatrix[0].length; j++){
                    System.out.print(RBTMatrix[i][j] + " ");
                }
                System.out.println();
            }
            */
            double[] trannyHomoPoint = LinAlg.matrixAB(RBTMatrix, homoPoint);
            double[] trannyPoint = new double[]{trannyHomoPoint[0], trannyHomoPoint[1]};

            transformedPoints.add(trannyPoint);

        }
        return transformedPoints;
    }

}