package team;

public class LandEdge extends Edge{


    public LandEdge(int jacobStartRow, int firstBlockColumnStart, int secondBlockColumnStart) {
        
        this.jacobianStartRow = jacobStartRow;
        this.block1Column = firstBlockColumnStart;
        this.block2Column = secondBlockColumnStart;
        
    }

    public JacobBlock getJacob(double[] theStateVector){
	
        // Create a new JacobBlock class
        JacobBlock myJacobBlock = new JacobBlock(jacobianStartRow, block1Column, block2Column);

        double x0;  
        double y0;  
        double l_x; 
        double l_y; 
					

        if (this.node1.isLand()) {
           
            x0  = theStateVector[this.node2.getStateVectorIndex()];
			y0	= theStateVector[this.node2.getStateVectorIndex()+1]; 
			l_x = theStateVector[this.node1.getStateVectorIndex()];
			l_y = theStateVector[this.node1.getStateVectorIndex()+1];

        } else {

            x0	= theStateVector[this.node1.getStateVectorIndex()];
			y0	= theStateVector[this.node1.getStateVectorIndex()+1]; 
			l_x = theStateVector[this.node2.getStateVectorIndex()];
			l_y = theStateVector[this.node2.getStateVectorIndex()+1];

        }

        // Jacobian with respect to a position
        // Create block one ... should be 2x3
        double[][] firstBlock = new double[2][3];
        firstBlock[0][0] = -(2*l_x - 2*x0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        firstBlock[0][1] = -(2*l_y - 2*y0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        firstBlock[0][2] =  0;
        firstBlock[1][0] =  (l_y - y0)/(Math.pow(l_x - x0,2)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));
        firstBlock[1][1] =  -1/((l_x - x0)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));
        firstBlock[1][2] =  -1;

        myJacobBlock.setFirstBlock(firstBlock);


        // Jacobian with respect to a landmark
        // Create block two  ... should be 2x2
        double[][] secondBlock = new double[2][2];        
        secondBlock[0][0] =  (2*l_x - 2*x0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        secondBlock[0][1] =   (2*l_y - 2*y0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        secondBlock[1][0] =   -(l_y - y0)/(Math.pow(l_x - x0,2)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));
        secondBlock[1][1] =   1/((l_x - x0)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));

        myJacobBlock.setSecondBlock(secondBlock);

        return myJacobBlock;

    }

    public double [] getResiduals() {
        return null;
    }

    public CovBlock getCovBlock() {
        return null;
    }


}