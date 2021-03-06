/** EECS 568 Homework 2, SLAM
Steve Chaves
Schuyler Cohen
Patrick O'Keefe
Paul Ozog
**/


package team.PS2;

import java.util.*;

public class LandEdge extends Edge{


    public LandEdge(int jacobStartRow, int firstBlockColumnStart, int secondBlockColumnStart, Node nodeOne, Node nodeTwo) {

        this.jacobianStartRow = jacobStartRow;
        this.block1Column = firstBlockColumnStart;
        this.block2Column = secondBlockColumnStart;

        this.node1 = nodeOne;
        this.node2 = nodeTwo;

    }

    public JacobBlock getJacob(List<Node> theStateVector){

        // Create a new JacobBlock class
        JacobBlock myJacobBlock = new JacobBlock(jacobianStartRow, block1Column, block2Column);

        double x0;
        double y0;
        double l_x;
        double l_y;

        double [] status2 = theStateVector.get(this.node2.getStateVectorIndex()).getState();
        double [] status1 = theStateVector.get(this.node1.getStateVectorIndex()).getState();
        if (this.node1.isLand()) {


            x0  = status2[0];
            y0  = status2[1];
            l_x = status1[0];
            l_y = status1[1];

        } else {

            x0  = status1[0];
            y0  = status1[1];
            l_x = status2[0];
            l_y = status2[1];

        }

        // Jacobian with respect to a position
        // Create block one ... should be 2x3
        double[][] firstBlock = new double[2][3];
        firstBlock[0][0] = -(2*l_x - 2*x0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        firstBlock[0][1] = -(2*l_y - 2*y0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        firstBlock[0][2] =  0;
        firstBlock[1][0] =  (l_y - y0)/(Math.pow(l_x - x0,2)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));
        firstBlock[1][1] =  -1/((l_x - x0)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));
        firstBlock[1][2] =  -1;

        myJacobBlock.setFirstBlock(firstBlock);


        // Jacobian with respect to a landmark
        // Create block two  ... should be 2x2
        double[][] secondBlock = new double[2][2];
        secondBlock[0][0] =  (2*l_x - 2*x0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        secondBlock[0][1] =   (2*l_y - 2*y0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        secondBlock[1][0] =   -(l_y - y0)/(Math.pow(l_x - x0,2)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));
        secondBlock[1][1] =   1/((l_x - x0)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));

        myJacobBlock.setSecondBlock(secondBlock);

        return myJacobBlock;

    }

    public double [] getResiduals() {
        return null;
    }

    /**
     * getCovBlock
     *
     * Note: these parameters are unused
     */
    public CovBlock getCovBlock(double t_l, double t_r) {

        double landmarkSig[] = config.requireDoubles("noisemodels.landmarkDiag");

        double[][] result = new double[2][2];
        result[0][0] = Math.pow(landmarkSig[0], 2);
        result[0][1] = 0.0;
        result[1][0] = 0.0;
        result[1][1] = Math.pow(landmarkSig[1], 2);


        CovBlock theCov = new CovBlock(jacobianStartRow, jacobianStartRow);
        theCov.setTheBlock(result);

        return theCov;
    }


}
