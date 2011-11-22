package team.slam;

public abstract class Node{

    private double [] state;

    private int dof;

    public int getDOF(){
        return dof;
    }

    public Node(int inDof){
        dof = inDof;
        state = new double[dof];
    }
}
