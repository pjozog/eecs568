package team;


public abstract class Edge{

    public Node node1;
    public Node node2;

    public abstract JacobBlock getJacob(double[] theStateVector);

    public abstract double [] getResiduals();

    public abstract CovBlock getCovBlock();


}

