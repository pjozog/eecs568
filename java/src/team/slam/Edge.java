package team.slam;

public abstract class Edge{
    
    private Node node1;
    private Node node2;

    public Edge(Node n1, Node n2){
        node1 = n1;
        node2 = n2;
    }
}