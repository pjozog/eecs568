package team;

import april.jmat.*;

public class JacobBlock{


    private double [][] block1;
    private double [][] block2;
    private int row;
    private int block1Col;
    private int block2Col;

    public static Matrix assemble(ArrayList<JacobBlock> blocks){


	return null;
    }

    public int getRow(){
	return row;
    }

    public int getFirstColumn(){
	return block1Col;
    }

    public int getSecondColum(){
	return block2Col;
    }

    public double[][] getFirstBlock(){
	return block1;
    }

    public double[][] getSecondBlock(){
	return block2;
    }

    
}