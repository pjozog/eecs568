package team;

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

            toReturn.set(block.getRow(), block.getColumn(), LinAlg.inverse(block.getBlock()));
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