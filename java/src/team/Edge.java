package team;


public abstract class Edge{

    public Node node1;
    public Node node2;

    public static Config config;

    protected int jacobianStartRow;
    protected int block1Column;
    protected int block2Column;

    public abstract JacobBlock getJacob(double[] theStateVector);

    public abstract double [] getResiduals();

    public abstract CovBlock getCovBlock();


}

