package team;

import april.jmat.*;
import java.util.*;

public class JacobBlock{


    private double [][] block1;
    private double [][] block2;
    private int row;
    private int block1Col;
    private int block2Col;


    public JacobBlock(int rowStart, int blockOneCol, int blockTwoCol) {

        this.row = rowStart;
        this.block1Col = blockOneCol;
        this.block2Col = blockTwoCol;

    }

    /** rows is the size of the observation vector, columns is the size of the state vector**/
    public static Matrix assemble(int rows, int cols, ArrayList<JacobBlock> blocks, int numPinningRows, JacobBlock pinned){

        
        Matrix toReturn = new Matrix(rows + numPinningRows, cols, Matrix.SPARSE);
        toReturn.set(pinned.getRow(), pinned.getFirstColumn(),  pinned.getFirstBlock());
        toReturn.set(pinned.getRow(), pinned.getSecondColumn(), pinned.getSecondBlock());
        for(JacobBlock block : blocks){

            toReturn.set(block.getRow() + numPinningRows, block.getFirstColumn(),  block.getFirstBlock());
            toReturn.set(block.getRow() + numPinningRows, block.getSecondColumn(), block.getSecondBlock());

        }
        return toReturn;
    }


    public int getRow(){
        return row;
    }

    public int getFirstColumn(){
        return block1Col;
    }

    public int getSecondColumn(){
        return block2Col;
    }

    public double[][] getFirstBlock(){
        return block1;
    }

    public double[][] getSecondBlock(){
        return block2;
    }

    public void setFirstBlock(double[][] theBlock) {
        this.block1 = theBlock;
    }

    public void setSecondBlock(double[][] theBlock) {
        this.block2 = theBlock;
    }


}