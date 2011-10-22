package team;

import java.awt.*;
import java.util.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;
import april.sim.*;

public class PartOneListener implements Simulator.Listener
{

    VisWorld vw;
    Config config;
    private int nextJacobRowIndex = 0;

    private int nextAbsStateRowIndex = 0;

    double xyt[] = new double[3]; // dead reconning

    ArrayList<double[]> trajectory = new ArrayList<double[]>();

    ArrayList<Node> allObservations = new ArrayList<Node>();
    ArrayList<Node> stateVector = new ArrayList<Node>();
    ArrayList<Edge> allEdges = new ArrayList<Edge>();

    ArrayList<JacobBlock> jacobList = new ArrayList<JacobBlock>();
    ArrayList<CovBlock> covList = new ArrayList<CovBlock>();

    private static int numUpdates = 0;

    /*maps from the landmark id to the index in the state vector array*/
    HashMap<Integer, Integer> landmarksSeen = new HashMap<Integer, Integer>();
    double baseline;
    OdNode lastOdNode =null;
    ArrayList<Simulator.odometry_t> allTicks = new ArrayList<Simulator.odometry_t>();
    private int numConverge;

    public void init(Config _config, VisWorld _vw)
    {
        config  = _config;
        vw = _vw;
        Edge.config = _config;
        baseline = config.requireDouble("robot.baseline_m");
        numConverge = config.requireInt("simulator.numConverge");
        lastOdNode = new OdNode(0, nextAbsStateRowIndex, 0, 0, 0);
        nextAbsStateRowIndex += lastOdNode.stateLength();
        stateVector.add(new OdNode(0, 0, 0, 0, 0));


        // This is bullshit
        allObservations.add(new OdNode(0, 0, 0.0, 0.0, 0.0));
        stateVector.add(new OdNode(0, 0, 0, 0, 0));


        OdEdge odEdge = new OdEdge(nextJacobRowIndex, lastOdNode.getAbsIndex(), nextAbsStateRowIndex, lastOdNode, lastOdNode);
        // allEdges.add(odEdge);
        nextJacobRowIndex += lastOdNode.stateLength();

    }
    private ArrayList<Node> getPredictedObs(){

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
            predicted.add(new OdNode(0, 0, x[0], x[1], x[2]));

            /*TODO NOTE, ignoring index and id for nodes*/
            ArrayList<Integer> landmarkIndex = node.getLandmarksSeen();
            for(int i : landmarkIndex){
                Node landNode = stateVector.get(i);
                //TODO take out for "production"
                if(true){
                    assert(landNode.isLand());
                }
                double pos[] = node.getState();
                double xa = pos[0];
                double ya = pos[1];
                double p = pos[2];

                double posL[] = landNode.getState();
                double xl = posL[0];
                double yl = posL[1];

                double r     = Math.sqrt(Math.pow(xl - xa, 2) + Math.pow(yl - ya, 2));
                double theta = Math.atan2(yl - ya, xl - xa) - p;
                predicted.add(new LandNode(0, 0, r, theta, 0));
            }

        }
        return predicted;


    }


    /*Every time we see a node, update the row of the jacob,
      Every edge we make connects two nodes, the first column number is the index of the first node,
      second is second*/
    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets)
    {

        allTicks.add(odom);
        numUpdates++;
        DenseVec ticksXYT = TicksUtil.ticksToXYT(odom, baseline);

        double x = ticksXYT.get(0);
        double y = ticksXYT.get(1);
        double t = ticksXYT.get(2);;
        int nodeIndex = stateVector.size();

        /*new = old * obs*/
        double [] newPos = LinAlg.xytMultiply(lastOdNode.getState(), new double[]{x, y, t});
        newPos[2] = MathUtil.mod2pi(newPos[2]);

        /*adds global coords*/
        OdNode odNode = new OdNode(nodeIndex, nextAbsStateRowIndex, newPos[0], newPos[1], newPos[2]);
        stateVector.add(odNode);


        OdEdge odEdge = new OdEdge(nextJacobRowIndex, lastOdNode.getAbsIndex(), nextAbsStateRowIndex, lastOdNode, odNode);
        odEdge.setOdom(odom);
        allEdges.add(odEdge);

        nextJacobRowIndex += lastOdNode.stateLength();



        /*adds relative readings*/
        allObservations.add(new OdNode(nodeIndex, nextAbsStateRowIndex, x, y, t));


        nextAbsStateRowIndex += odNode.stateLength();
        /*save last node to get next global position*/
        lastOdNode = odNode;

        for(Simulator.landmark_t det: dets){

            /*If this is a duplicate*/
            if(landmarksSeen.containsKey(det.id)){

                /*Same index as the one we already found*/
                nodeIndex = landmarksSeen.get(det.id);

                /*add the observation, but not to the state vector*/
                /*NOTE i dont think the abs number matters for observations*/
                Node landNode = new LandNode(nodeIndex, -1, det.obs[0], det.obs[1], det.id);
                allObservations.add(landNode);
                Edge landEdge = new LandEdge(nextJacobRowIndex, lastOdNode.getAbsIndex(), stateVector.get(nodeIndex).getAbsIndex(), lastOdNode, landNode);
                // Hopefully the odom from a land edge is never used!
                landEdge.setOdom(odom);
                allEdges.add(landEdge);
                nextJacobRowIndex += landNode.stateLength();
                odNode.sawLandmark(nodeIndex);
                continue;
            }

            /*add the landmark to the state vector and note we saw it, use global coords*/
            nodeIndex = stateVector.size();
            double []pos = LandUtil.rThetaToXY(det.obs[0], det.obs[1], newPos[0], newPos[1], newPos[2]);

            Node landNode = new LandNode(nodeIndex, nextAbsStateRowIndex, pos[0], pos[1], det.id);
            odNode.sawLandmark(nodeIndex);
            Edge landEdge = new LandEdge(nextJacobRowIndex, lastOdNode.getAbsIndex(), nextAbsStateRowIndex, lastOdNode, landNode);
            landEdge.setOdom(odom);
            allEdges.add(landEdge);

            nextJacobRowIndex += landNode.stateLength();

            allObservations.add(new LandNode(nodeIndex, nextAbsStateRowIndex, det.obs[0], det.obs[1], det.id));
            nextAbsStateRowIndex += landNode.stateLength();
            stateVector.add(landNode);

            landmarksSeen.put(new Integer(det.id), new Integer(nodeIndex));
        }//end landmarks.



        jacobList = new ArrayList<JacobBlock>();
        covList = new ArrayList<CovBlock>();
        for(int i = 0; i < numConverge; i++){
            jacobList.clear();
            covList.clear();


            for (Edge edge : allEdges){
                jacobList.add(edge.getJacob(stateVector));
                Simulator.odometry_t myEdgeOdom = edge.getOdom();
                covList.add(edge.getCovBlock(myEdgeOdom.obs[0], myEdgeOdom.obs[1]));
            }

            Matrix J = JacobBlock.assemble(nextJacobRowIndex, nextAbsStateRowIndex, jacobList);
            Matrix sigmaInv = CovBlock.assembleInverse(nextJacobRowIndex, nextJacobRowIndex, covList);

            ArrayList<Node> predicted = getPredictedObs();
            // int pTot = 0;
            // int oTot = 0;

            ArrayList<Double> r = new ArrayList<Double>();

            /*Error checking*/
            for(int j = 0; j < predicted.size(); j++){
                double[] p = predicted.get(j).getState();
                double[] o = allObservations.get(j).getState();



                for (int k = 0; k < p.length; k++) {
                    r.add(new Double(o[k]-p[k]));
                }

                // pTot += predicted.get(j).stateLength();
                // oTot += allObservations.get(j).stateLength();
                // assert (predicted.get(j).isLand() == allObservations.get(j).isLand());
            }

            double [] realR = new double[r.size()];
            for (int asdf = 0; asdf < r.size(); asdf++) {
                realR[asdf] = r.get(asdf);
            }


            // System.out.println("Predicted is " + predicted.size() + " nodes long with " + pTot + " total values" );
            // System.out.println("Observation is " + allObservations.size() + " nodes long with " + oTot + " total values" );
            double [][] Jarray = J.copyArray();
            double [][] sigArray = sigmaInv.copyArray();
            double [][] A = LinAlg.matrixAtBC(Jarray, sigArray, Jarray);
            double [][] jtSig = LinAlg.matrixAtB(Jarray, sigArray);

            System.out.println("Size of jtSig: "+ jtSig.length + " " + jtSig[0].length + "\nSize of realR: " + realR.length);
            double [] b = LinAlg.matrixAB(jtSig, realR);

            Matrix identityPerturbation = Matrix.identity(A.length, A[0].length).times(100.0);

            // double [][] AInv = LinAlg.inverse(LinAlg.add(A, identityPerturbation.copyArray()));
            double [][] regularizedA = LinAlg.add(A, identityPerturbation.copyArray());

            CholeskyDecomposition myDecomp = new CholeskyDecomposition(new Matrix(regularizedA));
            Matrix answer = myDecomp.solve(Matrix.columnMatrix(b));

            // if (AInv == null) {
            //     System.out.println("WE SUCK AT REGULARIZAION!");
            //     LinAlg.print(LinAlg.add(A, identityPerturbation.copyArray()));
            //     System.out.println("WE SUCK AT REGULARIZAION!");
            // }

            // double [] deltaX = LinAlg.matrixAB(AInv,b);

            double [] deltaX = answer.copyAsVector();

            System.out.println("DELTAX LENGTH " +deltaX.length);

            LinAlg.print(deltaX);
            System.out.println("");
            // int index = 0;
            // for (Node node : stateVector){
            //     index+=node.addToState(deltaX, index);

            // }


        }

        // System.out.println("********State vector*********");
        // for(Node n : stateVector){
        //     System.out.println(n);

        // }


        xyt = LinAlg.xytMultiply(xyt, new double[]{x, y ,t});

        trajectory.add(LinAlg.resize(xyt,2));

        System.out.println("Update #" + numUpdates + " with odom " + odom.obs[0] +","+ odom.obs[1]
                           + "\n\tand " + dets.size() + " landmark observations"
                           + "\n\tand xyt: ");
        LinAlg.printTranspose(ticksXYT.getDoubles());
        System.out.println();

        drawDummy(dets);
    }


    public void drawDummy(ArrayList<Simulator.landmark_t> landmarks)
    {
        // Draw local Trajectory
        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-local");
            vb.addBack(new VisLines(new VisVertexData(trajectory),
                                    new VisConstantColor(new Color(160,30,30)),
                                    1.5, VisLines.TYPE.LINE_STRIP));
            vb.swap();
        }

        ArrayList<double[]> rpoints = new ArrayList<double[]>();
        rpoints.add(new double[]{-.3, .3});
        rpoints.add(new double[]{-.3, -.3});
        rpoints.add(new double[]{.45,0});

        // Probably should be replaced with student-code
        {
            VisWorld.Buffer vb = vw.getBuffer("robot-local");
            VisObject robot = new VisLines(new VisVertexData(rpoints),
                                           new VisConstantColor(Color.red),
                                           3,
                                           VisLines.TYPE.LINE_LOOP);


            double xyzrpy[] = new double[]{xyt[0], xyt[1], 0,
                                           0, 0, xyt[2]};
            vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
            vb.swap();
        }

        // Draw the landmark observations
        {
            VisWorld.Buffer vb = vw.getBuffer("landmarks-noisy");
            for (Simulator.landmark_t lmark : landmarks) {
                double[] obs = lmark.obs;
                ArrayList<double[]> obsPoints = new ArrayList<double[]>();
                obsPoints.add(LinAlg.resize(xyt,2));
                double rel_xy[] = {obs[0] * Math.cos(obs[1]), obs[0] *Math.sin(obs[1])};
                obsPoints.add(LinAlg.transform(xyt, rel_xy));
                vb.addBack(new VisLines(new VisVertexData(obsPoints),
                                        new VisConstantColor(lmark.id == -1? Color.gray : Color.cyan), 2, VisLines.TYPE.LINE_STRIP));
            }
            vb.swap();
        }


    }
}
