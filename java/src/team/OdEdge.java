package team;

import java.util.*;

public class OdEdge extends Edge{

    private int jacobianStartRow;
    private int block1Column;
    private int block2Column;


    public OdEdge(int jacobStartRow, int firstBlockColumnStart, int secondBlockColumnStart) {
        
        this.jacobianStartRow = jacobStartRow;
        this.block1Column = firstBlockColumnStart;
        this.block2Column = secondBlockColumnStart;
        
    }


    public JacobBlock getJacob(List<Node> theStateVector){
	
        // Create a new JacobBlock class
        JacobBlock myJacobBlock = new JacobBlock(jacobianStartRow, block1Column, block2Column);

	double [] status1 = theStateVector.get(this.node1.getStateVectorIndex()).getState();
	double [] status2 = theStateVector.get(this.node2.getStateVectorIndex()).getState();


        double x0  = status1[0];
        double y0  = status1[1];
        double phi0 = status1[2];
        double x1 = status2[0];
        double y1 = status2[1];
                                     
        // Jacobian with respect to a position
        // Create block one ... should be 3x3
        double[][] firstBlock = new double[3][3];
        firstBlock[0][0] = -Math.cos(phi0);
		firstBlock[0][1] = -Math.sin(phi0);
		firstBlock[0][2] = Math.sin(phi0)*(x0-x1) - Math.cos(phi0)*(y0-y1);
		firstBlock[1][0] = Math.sin(phi0);
		firstBlock[1][1] = -Math.cos(phi0);
		firstBlock[1][2] = Math.cos(phi0)*(x0 - x1) + Math.sin(phi0)*(y0 - y1);
		firstBlock[2][0] = 0.0;
		firstBlock[2][1] = 0.0;
		firstBlock[2][2] = -1.0;

        myJacobBlock.setFirstBlock(firstBlock);


        // Jacobian with respect to a position
        // Create block two  ... should be 3x3
        double[][] secondBlock = new double[3][3];         
        secondBlock[0][0] = Math.cos(phi0);
		secondBlock[0][1] = Math.sin(phi0);
		secondBlock[0][2] = 0.0;
		secondBlock[1][0] = -Math.sin(phi0);
		secondBlock[1][1] = Math.cos(phi0);
		secondBlock[1][2] = 0.0;
		secondBlock[2][0] = 0.0;
		secondBlock[2][1] = 0.0;
        secondBlock[2][2] = 1.0;


        myJacobBlock.setSecondBlock(secondBlock);

        return myJacobBlock;

    }

    public double [] getResiduals() {
        return null;
    }


    public CovBlock getCovBlock() {


 //    (b*sin((t_l - t_r)/b))/(2*(t_l - t_r)) + (cos((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)) - (b*sin((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)^2),

 //    (b*sin((t_l - t_r)/b))/(2*(t_l - t_r)) - (cos((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)) + (b*sin((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)^2)]


 //    (b*(t_l + t_r))/(2*(t_l - t_r)^2) - b/(2*(t_l - t_r)) - (sin((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)) + (b*cos((t_l - t_r)/b))/(2*(t_l - t_r)) - (b*cos((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)^2),

 // (sin((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)) - (b*(t_l + t_r))/(2*(t_l - t_r)^2) - b/(2*(t_l - t_r)) + (b*cos((t_l - t_r)/b))/(2*(t_l - t_r)) + (b*cos((t_l - t_r)/b)*(t_l + t_r))/(2*(t_l - t_r)^2)


 //  -1/b,

 //  1/b]
 

        return null;

    }




}