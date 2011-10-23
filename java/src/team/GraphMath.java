package team;

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