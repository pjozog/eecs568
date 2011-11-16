package team.PS2;

import april.jmat.*;
import java.util.*;
import april.sim.*;

public class GraphMath{

    /*can't make instance of*/
    private GraphMath(){}
    

    /**Assemble all of the Jacobian Blocks. 
       This re-evaluates the jacobian based on our current state vector.*/
    public static ArrayList<JacobBlock> getJacobList(ArrayList<Edge> edges, ArrayList<Node> stateVector){
        ArrayList<JacobBlock> jacob = new ArrayList<JacobBlock>();
        
        for (Edge edge : edges){
            jacob.add(edge.getJacob(stateVector));
        }
        return jacob;
    }

    public static Matrix calcFASTDeltaX(ArrayList<Edge> allEdges,
                                        ArrayList<Node> observed,
                                        double lambda,
                                        boolean pin,
                                        ArrayList<Node> stateVector,
                                        int nextAbsStateRowIndex,
                                        JacobBlock pinningJacob,
                                        CovBlock pinningCov) {
        

        double[] realR = getResiduals(stateVector, observed, pin);
        
        Matrix A = new Matrix(nextAbsStateRowIndex, nextAbsStateRowIndex, Matrix.SPARSE);
        // Adding 3 for the pinned node

        Matrix b = new Matrix(nextAbsStateRowIndex, 1);

        // Fake the "edge" for the first pinned node
        double [][] JtSigPinned = LinAlg.matrixAtB(pinningJacob.getFirstBlock(), pinningCov.getBlock());

        A.plusEquals(0, 0, LinAlg.matrixAB(JtSigPinned, pinningJacob.getFirstBlock()));

        double[] pinnedResidual = new double[3];
        pinnedResidual[0] = realR[0];
        pinnedResidual[1] = realR[1];
        pinnedResidual[2] = realR[2];

        b.plusEqualsColumnVector(0, 0, LinAlg.matrixAB(JtSigPinned, pinnedResidual));

        for (Edge edge: allEdges) {
            JacobBlock thisEdgeJ = edge.getJacob(stateVector);
                
            // These two nodes will effect four parts of A
            int stateIndexOne = edge.node1.getAbsIndex();
            int stateIndexTwo = edge.node2.getAbsIndex();

            double [] edgeZOfX;
            if (edge.isLandmarkEdge()) {
                edgeZOfX =  new double[2];
                edgeZOfX[0] = realR[edge.getJacobianStartRow()+3];
                edgeZOfX[1] = realR[edge.getJacobianStartRow()+1+3];
            } else {
                edgeZOfX =  new double[3];
                edgeZOfX[0] = realR[edge.getJacobianStartRow()+3];
                edgeZOfX[1] = realR[edge.getJacobianStartRow()+1+3];
                edgeZOfX[2] = realR[edge.getJacobianStartRow()+2+3];
            }

                
                
            Simulator.odometry_t myEdgeOdom = edge.getOdom();
            CovBlock theCovBlock = edge.getCovBlock(myEdgeOdom.obs[0], myEdgeOdom.obs[1]);


            double [][] JtW;
            JtW = LinAlg.matrixAtB(thisEdgeJ.getFirstBlock(), LinAlg.inverse(theCovBlock.getBlock()));


            double [][]JtWJ = LinAlg.matrixAB(JtW, thisEdgeJ.getFirstBlock());
                
            // Add contribution of node 1 and node 1
            A.plusEquals(stateIndexOne, stateIndexOne, JtWJ);
                
            JtWJ = LinAlg.matrixAB(JtW, thisEdgeJ.getSecondBlock());

            // Add contribution of node 1 and node 2
            A.plusEquals(stateIndexOne, stateIndexTwo, JtWJ);

            // Add contribution of node 1 to jtSig
            // jtSig.plusEquals(stateIndexOne, edge.getJacobianStartRow() +3 , JtW);
            b.plusEqualsColumnVector(stateIndexOne, 0, LinAlg.matrixAB(JtW, edgeZOfX));

            JtW = LinAlg.matrixAtB(thisEdgeJ.getSecondBlock(), LinAlg.inverse(theCovBlock.getBlock()));

            JtWJ = LinAlg.matrixAB(JtW, thisEdgeJ.getFirstBlock());

            // Add contribution of node 2 and node 1
            A.plusEquals(stateIndexTwo, stateIndexOne, JtWJ);

         
            JtWJ = LinAlg.matrixAB(JtW, thisEdgeJ.getSecondBlock());

            // Add contribution of node 2 and node 2
            A.plusEquals(stateIndexTwo, stateIndexTwo, JtWJ);

            // Add contribution of node 2 to jtSig
            // jtSig.plusEquals(stateIndexTwo, edge.getJacobianStartRow() +3 , JtW);
            b.plusEqualsColumnVector(stateIndexTwo, 0, LinAlg.matrixAB(JtW, edgeZOfX));


        }


        A = A.plus(Matrix.identity(A.getRowDimension(), A.getColumnDimension()).times(lambda));

        assert(A.isSparse());
        
        CholeskyDecomposition myDecomp = new CholeskyDecomposition(A);
        
        Matrix deltaX = myDecomp.solve(b);

        return deltaX;

    }

    public static Matrix calcDeltaX(Matrix J, Matrix sigmaInv, ArrayList<Node> observed, double lambda, boolean pin, ArrayList<Node> stateVector){
    
        assert(J.isSparse() && sigmaInv.isSparse());

        Matrix jtSig = J.transpose().times(sigmaInv);
        assert(jtSig.isSparse());
        
        Matrix A = jtSig.times(J);
        assert(A.isSparse());
        
        Matrix b = jtSig.times(Matrix.columnMatrix(getResiduals(stateVector, observed, pin)));
        
        A = A.plus(Matrix.identity(A.getRowDimension(), A.getColumnDimension()).times(lambda));
        assert(A.isSparse());
        
        CholeskyDecomposition myDecomp = new CholeskyDecomposition(A);
        
        Matrix deltaX = myDecomp.solve(b);
        return deltaX;
       
    }


    public static double[] getResiduals(ArrayList<Node> stateVector, ArrayList<Node> observed, boolean pin){

        ArrayList<Node> predicted = getPredictedObs(stateVector);

        ArrayList<Double> r = new ArrayList<Double>();

        //*Get residuals*/
        for(int j = 0; j < predicted.size(); j++){
            double[] p = predicted.get(j).getState();
            double[] o = observed.get(j).getState();
            if(!predicted.get(j).isLand()){
                p[2] = MathUtil.mod2pi(p[2]);
                o[2] = MathUtil.mod2pi(o[2]);
            }

            for (int k = 0; k < p.length; k++) {
                r.add(new Double(o[k]-p[k]));
            }
        }


        /*includes 3 zeros at top for pinning*/
        int numZeros = 3;
        if(!pin){
            numZeros = 0;
        }
        double [] realR = new double[r.size() + numZeros];
        for (int k = 0; k < numZeros; k++){
            realR[k] = 0;
        }
        for (int k = 0; k < r.size(); k++) {
            realR[k + numZeros] = r.get(k);
        }
       
        return realR;

    }


    public static ArrayList<Node> getPredictedObs(ArrayList<Node> stateVector){

        ArrayList<Node> predicted = new ArrayList<Node>();

        /*ignore landmarks.
          for all od measurements, subtract (matrix version) from previous od measurement
          then calculate distance to all landmarks*/
        Node lastOdNode = stateVector.get(0);
        for (Node node : stateVector.subList(1, stateVector.size())){

            if(node.isLand()){
                continue;
            }
            /*obs = old -^1 * new*/
            double []x = LinAlg.xytInvMul31(lastOdNode.getState(), node.getState());
            predicted.add(new OdNode(0, 0, x[0], x[1], MathUtil.mod2pi(x[2])));

            /*TODO NOTE, ignoring index and id for nodes*/
            ArrayList<Integer> landmarkIndex = node.getLandmarksSeen();
            for(int i : landmarkIndex){
                Node landNode = stateVector.get(i);

                double pos[] = node.getState();
                double xa = pos[0];
                double ya = pos[1];
                double p = MathUtil.mod2pi(pos[2]);

                double posL[] = landNode.getState();
                double xl = posL[0];
                double yl = posL[1];

                double r     = Math.sqrt(Math.pow(xl - xa, 2) + Math.pow(yl - ya, 2));
                double theta = Math.atan2(yl - ya, xl - xa) - p;
                predicted.add(new LandNode(0, 0, r, MathUtil.mod2pi(theta), 0));
            }
            lastOdNode = node;

        }
        return predicted;
    }

    public static Matrix addToSigmaInverse(Matrix oldSigmaInv, int numEdgesToAdd, ArrayList<Edge> edges, int size, boolean pin){
       
        ArrayList<CovBlock> covList = new ArrayList<CovBlock>();
        for (int j = numEdgesToAdd; j > 0; j --) {
            Edge edge = edges.get(edges.size()-j);
            Simulator.odometry_t edgeOdom = edge.getOdom();
            covList.add(edge.getCovBlock(edgeOdom.obs[0], edgeOdom.obs[1]));
        }

        Matrix newSigmaInv = CovBlock.assembleInverse(size, size, covList, 3, null, pin);
        newSigmaInv.set(0, 0, oldSigmaInv.copyArray());
        return newSigmaInv;
    }

}