/** EECS 568 Homework 2, SLAM
Steve Chaves
Schuyler Cohen
Patrick O'Keefe
Paul Ozog
**/


package team;

import april.jmat.*;
import java.util.*;

public class CovBlock{

    private double [][] theBlock;
    private int row;
    private int blockCol;


    public CovBlock(int rowStart, int blockColumn) {
        this.row = rowStart;
        this.blockCol = blockColumn;
    }



    public static Matrix assembleInverse(int rows, int cols, ArrayList<CovBlock> blocks, int numPinningRows, CovBlock pinnedBlock) {

        Matrix toReturn = new Matrix(rows +numPinningRows, cols + numPinningRows, Matrix.SPARSE);

        toReturn.set(pinnedBlock.getRow(), pinnedBlock.getColumn(), LinAlg.inverse(pinnedBlock.getBlock()));
        for (CovBlock block : blocks) {


            // Perform inversion on our diagonal block
            double [][] blockInv = LinAlg.inverse(block.getBlock());

            // Make sure it's not null! It shouldn't be if we do our projections right for odometry
            assert (blockInv != null);

            // Add this block in the proper place in the grand covariance matrix
            toReturn.set(block.getRow() + numPinningRows, block.getColumn() + numPinningRows, blockInv);


        }

        //System.out.println("********");
        //LinAlg.printPattern(toReturn.copyArray());
        //System.out.println("********************");

        return toReturn;

    }

    public int getRow() {
        return row;
    }

    public int getColumn() {
        return blockCol;
    }

    public double[][] getBlock() {
        return theBlock;
    }

    public void setTheBlock(double[][] newBlock) {
        this.theBlock = newBlock;
    }

}
