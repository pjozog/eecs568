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



    public static Matrix assembleInverse(int rows, int cols, ArrayList<CovBlock> blocks) {

        Matrix toReturn = new Matrix(rows, cols, Matrix.SPARSE);

        for (CovBlock block : blocks) {
            
            // assert(LinAlg.inverse(block.getBlock())!=null);
            // System.out.println("ASDF********");
            // LinAlg.print(block.getBlock());
            // System.out.println("ASDF********");
            // System.out.println(LinAlg.inverse(block.getBlock()));

            // Perform inversion on our diagonal block
            double [][] blockInv = LinAlg.inverse(block.getBlock());

            // Make sure it's not null! It shouldn't be if we do our projections right for odometry
            assert (blockInv != null);

            // Add this block in the proper place in the grand covariance matrix
            toReturn.set(block.getRow(), block.getColumn(), blockInv);


        }

        System.out.println("********");
        LinAlg.printPattern(toReturn.copyArray());
        System.out.println("********************");

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