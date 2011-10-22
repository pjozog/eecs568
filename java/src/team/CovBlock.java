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
            // LinAlg.print(LinAlg.inverse(block.getBlock()));
            // System.out.println(LinAlg.inverse(block.getBlock()));

            if (LinAlg.inverse(block.getBlock()) == null) {
                double [][] ident = new double[3][3];

                for (int i=0; i < 3; i++) {
                    for (int j =0; j < 3; j++) {
                        if (i == j) {
                            ident[i][j] = 1;
                        } else {
                            ident[i][j] = 0;
                        }
                    }
                }


                toReturn.set(block.getRow(), block.getColumn(), ident);

            } else {
                toReturn.set(block.getRow(), block.getColumn(), LinAlg.inverse(block.getBlock()));
            }


        }

        // System.out.println("********");
        // LinAlg.printPattern(toReturn.copyArray());
        // System.out.println("********************");

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