/** EECS 568 Homework 2, SLAM
    Steve Chaves
    Schuyler Cohen
    Patrick O'Keefe
    Paul Ozog
**/


package team.PS2;

import java.util.*;
import april.jmat.*;

public class OdEdge extends Edge{


    public OdEdge(int jacobStartRow, int firstBlockColumnStart, int secondBlockColumnStart, Node nodeOne, Node nodeTwo) {

        this.jacobianStartRow = jacobStartRow;
        this.block1Column = firstBlockColumnStart;
        this.block2Column = secondBlockColumnStart;

        this.node1 = nodeOne;
        this.node2 = nodeTwo;

    }


    public JacobBlock getJacob(List<Node> theStateVector){

        // Create a new JacobBlock class
        JacobBlock myJacobBlock = new JacobBlock(jacobianStartRow, block1Column, block2Column);

        double [] status1 = theStateVector.get(this.node1.getStateVectorIndex()).getState();
        double [] status2 = theStateVector.get(this.node2.getStateVectorIndex()).getState();


        double x0  = status1[0];
        double y0  = status1[1];
        double phi0 = status1[2];
        double x1 = status2[0];
        double y1 = status2[1];

        // Jacobian with respect to a position
        // Create block one ... should be 3x3
        double[][] firstBlock = new double[3][3];
        firstBlock[0][0] = -Math.cos(phi0);
        firstBlock[0][1] = -Math.sin(phi0);
        firstBlock[0][2] = Math.sin(phi0)*(x0-x1) - Math.cos(phi0)*(y0-y1);
        firstBlock[1][0] = Math.sin(phi0);
        firstBlock[1][1] = -Math.cos(phi0);
        firstBlock[1][2] = Math.cos(phi0)*(x0 - x1) + Math.sin(phi0)*(y0 - y1);
        firstBlock[2][0] = 0.0;
        firstBlock[2][1] = 0.0;
        firstBlock[2][2] = -1.0;

        myJacobBlock.setFirstBlock(firstBlock);


        // Jacobian with respect to a position
        // Create block two  ... should be 3x3
        double[][] secondBlock = new double[3][3];
        secondBlock[0][0] = Math.cos(phi0);
        secondBlock[0][1] = Math.sin(phi0);
        secondBlock[0][2] = 0.0;
        secondBlock[1][0] = -Math.sin(phi0);
        secondBlock[1][1] = Math.cos(phi0);
        secondBlock[1][2] = 0.0;
        secondBlock[2][0] = 0.0;
        secondBlock[2][1] = 0.0;
        secondBlock[2][2] = 1.0;


        myJacobBlock.setSecondBlock(secondBlock);

        return myJacobBlock;

    }

    public double [] getResiduals() {
        return null;
    }


    public CovBlock getCovBlock(double t_l, double t_r) {


        double b = config.requireDouble("robot.baseline_m");
        double odomD[] = config.requireDoubles("noisemodels.odometryDiag");
        double sigmaL = odomD[0];
        double sigmaR = odomD[1];


        double[][] tltrCovariance = new double[2][2];
        tltrCovariance[0][0] = Math.pow(t_l*sigmaL, 2);
        tltrCovariance[0][1] = 0.0;
        tltrCovariance[1][0] = 0.0;
        tltrCovariance[1][1] = Math.pow(t_r*sigmaR, 2);

        double[][] tltrToXYTJacob = new double[3][2];

        //        if (t_l == t_r) {
        /*if (true) {

          tltrToXYTJacob[0][0] = 1.0;
          tltrToXYTJacob[0][1] = 1.0;
          //TODO: I changed this
          tltrToXYTJacob[1][0] = 1.1;
          tltrToXYTJacob[1][1] = 1.0;
          tltrToXYTJacob[2][0] = -1/b;
          tltrToXYTJacob[2][1] = 1/b;


          } else {


          tltrToXYTJacob[0][0] =  (b*Math.sin((t_l - t_r)/b))/(2*(t_l - t_r)) + (Math.cos((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r))
          - (b*Math.sin((t_l - t_r)/b)*(t_l + t_r))/(2*Math.pow(t_l - t_r,2));

          tltrToXYTJacob[0][1] =  (b*Math.sin((t_l - t_r)/b))/(2*(t_l - t_r)) - (Math.cos((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r))
          + (b*Math.sin((t_l - t_r)/b)*(t_l + t_r))/(2*Math.pow(t_l - t_r,2));

          tltrToXYTJacob[1][0] = (b*(t_l + t_r))/(2*Math.pow(t_l - t_r,2)) - b/(2*(t_l - t_r))
          - (Math.sin((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)) + (b*Math.cos((t_l - t_r)/b))/(2*(t_l - t_r))
          - (b*Math.cos((t_l - t_r)/b)*(t_l + t_r))/(2*Math.pow(t_l - t_r,2));

          tltrToXYTJacob[1][1] = (Math.sin((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)) - (b*(t_l + t_r))/(2*Math.pow(t_l - t_r,2))
          - b/(2*(t_l - t_r)) + (b*Math.cos((t_l - t_r)/b))/(2*(t_l - t_r))
          + (b*Math.cos((t_l - t_r)/b)*(t_l + t_r))/(2*Math.pow(t_l - t_r,2));

          tltrToXYTJacob[2][0] = -1/b;
          tltrToXYTJacob[2][1] = 1/b;

          }*/

        //Trying different Jacobian...
        double dPhi = MathUtil.mod2pi(Math.atan2(t_r - t_l, b));
        // double s  = (t_l + t_r)/(2.0);
        double x = (t_l + t_r)/(2.0);

        // tltrToXYTJacob[0][0] = (Math.cos(dPhi)/2.0) + ((s*Math.sin(dPhi))/2.0);
        // tltrToXYTJacob[0][1] = (Math.cos(dPhi)/2.0) - ((s*Math.sin(dPhi))/2.0);
        // tltrToXYTJacob[1][0] = (Math.sin(dPhi)/2.0) - ((s*Math.cos(dPhi))/2.0);
        // tltrToXYTJacob[1][1] = (Math.sin(dPhi)/2.0) + ((s*Math.cos(dPhi))/2.0);
        // tltrToXYTJacob[2][0] = -1/b;
        // tltrToXYTJacob[2][1] = 1/b;

        tltrToXYTJacob[0][0] = .5;
        tltrToXYTJacob[0][1] = .5;
        tltrToXYTJacob[1][0] = 0.0;
        tltrToXYTJacob[1][1] = 0.0;
        tltrToXYTJacob[2][0] = -1.0/(b*(Math.pow(t_l - t_r, 2)/Math.pow(b,2) + 1));
        tltrToXYTJacob[2][1] =  1.0/(b*(Math.pow(t_l - t_r, 2)/Math.pow(b,2) + 1));



        double[][] result = new double[3][3];
        result = LinAlg.matrixABCt(tltrToXYTJacob, tltrCovariance, tltrToXYTJacob);
        result[1][1] = result[0][0]/100.0;


        // Hack in case we're always getting singular stuff during inversion
        // result[0][0] = Math.pow(t_l*sigmaL, 2);;
        // result[0][1] = 0.0;
        // result[0][2] = 0.0;
        // result[1][0] = 0.0;
        // result[1][1] = Math.pow(t_l*sigmaL, 2);
        // result[1][2] = 0.0;
        // result[2][0] = 0.0;
        // result[2][1] = 0.0;
        // result[2][2] = Math.pow(t_l*sigmaL, 2);




        CovBlock theCov = new CovBlock(jacobianStartRow, jacobianStartRow);
        theCov.setTheBlock(result);


        return theCov;

    }




}
