package team;

import april.jmat.*;
import java.util.*;

public class JacobBlock{


    private double [][] block1;
    private double [][] block2;
    private int row;
    private int block1Col;
    private int block2Col;

    /** rows is the size of the observation vector, columns is the size of the state vector**/
    public static Matrix assemble(int rows, int cols, ArrayList<JacobBlock> blocks){
	
	Matrix toReturn = new Matrix(rows, cols, Matrix.SPARSE);
	
	for(JacobBlock block : blocks){
	    
	    toReturn.set(block.getRow(), block.getFirstColumn(),  block.getFirstBlock());
	    toReturn.set(block.getRow(), block.getSecondColumn(), block.getSecondBlock());
	
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

    
}