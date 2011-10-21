package team;


public abstract class Edge{

    public Node node1;
    public Node node2;

    public abstract JacobBlock getJacob();

    public double [] getResiduals(){
	/*TODO Write*/
	return null;
    }

    public CovBlock getCovBlock(){
	/*TODO Write*/
	return null;
    }

}

