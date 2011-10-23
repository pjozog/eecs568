/** EECS 568 Homework 2, SLAM
    Steve Chaves
    Schuyler Cohen
    Patrick O'Keefe
    Paul Ozog
**/


package team;

import java.awt.*;
import java.util.*;
import java.io.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;
import april.sim.*;

public class PartOneListener implements Simulator.Listener
{

    VisWorld vw;
    Config config;

    /*id to assigin to next unknown landmark*/
    private static int nextLandmarkId = 1;

    private int nextJacobRowIndex = 0;

    private int nextAbsStateRowIndex = 0;

    private double curChi2Error = 0;

    /*TODO is this percent? absolute?*/
    private double chi2Thresh = 10;

    double xyt[] = new double[3]; // dead reconning

    private ArrayList<double[]> trajectory                 = new ArrayList<double[]>();
    private ArrayList<double[]> estimatedLandmarkPositions = new ArrayList<double[]>();
    private ArrayList<Node> allObservations                = new ArrayList<Node>();
    private ArrayList<Node> stateVector                    = new ArrayList<Node>();
    private ArrayList<Edge> allEdges                       = new ArrayList<Edge>();
    private ArrayList<JacobBlock> jacobList                = new ArrayList<JacobBlock>();
    private ArrayList<CovBlock> covList                    = new ArrayList<CovBlock>();
    private ArrayList<Simulator.odometry_t> allTicks       = new ArrayList<Simulator.odometry_t>();

    /*landmark ID to statevector index*/
    private HashMap<Integer, Integer> landmarksSeen        = new HashMap<Integer, Integer>();

    private Matrix cumulativeSigmaInverse;
    private int numEdgesToAddToSigma = 0;

    private JacobBlock pinningJacob = new JacobBlock(0, 0, 3);
    private CovBlock pinningCov = new CovBlock(0, 0);

    private int numPinnedRows = 3;
    private int debug = 0;
    private static int numUpdates = 0;
    private boolean landmarksHaveId;

    private double baseline;
    private OdNode lastOdNode =null;

    private int numConverge;
    private double lambda;
    private double landmarkDistThresh;
    private boolean pin = true;

    private long startTime;
    private int numErrors = 0;


    // private ArrayList<double[]> givenLandmarks = new ArrayList<double[]>();
    private HashMap<Integer, Integer> mapMyIdToRealId = new HashMap<Integer, Integer>();

    public void init(Config _config, VisWorld _vw)
    {
        config  = _config;
        vw = _vw;
        Edge.config = _config;

        baseline           = config.requireDouble("robot.baseline_m");
        numConverge        = config.requireInt("simulator.numConverge");
        lambda             = config.requireDouble("simulator.lambda");
        debug              = config.requireInt("simulator.debug");
        landmarksHaveId    = config.requireBoolean("simulator.knownDataAssoc");
        landmarkDistThresh = config.requireDouble("simulator.landmarkThresh");
        chi2Thresh         = config.requireDouble("simulator.chi2Thresh");
        /*
        for(int i = 0; ; i++){
            double li[] = config.getDoubles("landmarks.l" + i, null);
            if(li == null){
                break;
            }
            givenLandmarks.add(li);

        }
        for(double [] li : givenLandmarks){
            for(int i = 0; i < li.length; i++){
                System.out.println(li[i] + " ");
            }
            System.out.println();
        }
        */
        OdNode initial = new OdNode(0, nextAbsStateRowIndex, 0, 0, 0);
        lastOdNode = initial;
        nextAbsStateRowIndex += lastOdNode.stateLength();
        stateVector.add(initial);

        pinningJacob.setFirstBlock( new double[][]{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} });
        pinningJacob.setSecondBlock(new double[][]{ {0, 0, 0}, {0, 0, 0}, {0, 0, 0} });
        pinningCov.setTheBlock(new double [][]{ {100, 0, 0}, {0, 100, 0}, {0, 0, 100}});

        cumulativeSigmaInverse = new Matrix(3, 3, Matrix.SPARSE);
        cumulativeSigmaInverse.set(0, 0, new double [][]{ {100, 0, 0}, {0, 100, 0}, {0, 0, 100}});

        startTime = System.nanoTime();

    }
    /** tries to assoiciate landmark with seen landmarks,
        if no landmark is seen, creates a new id
        x, y are guessed global coordinates of observed landmark

    **/
    private int getLandmarkId(Node currentPos, Simulator.landmark_t det, Simulator.odometry_t odom, Node lastOdNode){

        assert(!currentPos.isLand());

        int returnVal = nextLandmarkId;
        double minDist = 999;
        double minChi2Error = 500000;
        boolean forceOldLandmark = false;

        for (Map.Entry<Integer, Integer> entry : landmarksSeen.entrySet()) {
           
            Integer ID = entry.getKey();
            Integer nodeIndex = entry.getValue();

            ArrayList<Edge> trialEdges = new ArrayList<Edge>(allEdges);
            ArrayList<Node> trialObs   = new ArrayList<Node>(allObservations);

            /*add the observation, but not to the state vector*/
            /*NOTE prettr sure the node index of the land node doesnt matter, only used in trial obs
              TODO is this true?*/
            Node landNode = new LandNode(nodeIndex,  stateVector.get(nodeIndex).getAbsIndex(), det.obs[0], MathUtil.mod2pi(det.obs[1]), ID);
            trialObs.add(landNode);

            Edge landEdge = new LandEdge(nextJacobRowIndex, lastOdNode.getAbsIndex(), stateVector.get(nodeIndex).getAbsIndex(), lastOdNode, landNode);

            // Hopefully the odom from a land edge is never used!
            landEdge.setOdom(odom);
            trialEdges.add(landEdge);

            int trialJacobRowIndex = nextJacobRowIndex + landNode.stateLength();

            boolean alreadySawLandmark = currentPos.seenLandmark(nodeIndex);

            currentPos.sawLandmark(nodeIndex);

            /*TODO is this 1?, maybe jacob rwo index should be increased?*/

            Matrix newSigmaInverse = GraphMath.addToSigmaInverse(cumulativeSigmaInverse, numEdgesToAddToSigma + 1, trialEdges, trialJacobRowIndex, pin);//trialSigmaInverse.copy();

            assert(newSigmaInverse.isSparse());

            ArrayList<JacobBlock> jacobs = GraphMath.getJacobList(trialEdges, stateVector);

            Matrix J = JacobBlock.assemble(trialJacobRowIndex, nextAbsStateRowIndex, jacobs, numPinnedRows, pinningJacob, pin);


            // Matrix deltaX  = GraphMath.calcDeltaX(J, newSigmaInverse, trialObs, lambda, pin, stateVector);
            Matrix deltaX = GraphMath.calcFASTDeltaX(trialEdges, trialObs, lambda, pin, stateVector,nextAbsStateRowIndex, pinningJacob, pinningCov);

            double [] trialResiduals = GraphMath.getResiduals(stateVector, trialObs, pin);


            double chi2Error = getChi2Error(J, deltaX, trialResiduals, newSigmaInverse);
            
            double [] newPos = currentPos.getState();
            double []pos = LandUtil.rThetaToXY(det.obs[0], det.obs[1], newPos[0], newPos[1], newPos[2]);
            double x_global = pos[0];
            double y_global = pos[1];
            Node node = stateVector.get(entry.getValue());
   
            assert(node.isLand());
   
            
            double state[] = node.getState();
            double dist = Math.sqrt(Math.pow(x_global - state[0], 2) + Math.pow(y_global - state[1], 2));
            if(dist < landmarkDistThresh){
                forceOldLandmark = true;
            }


            if((chi2Error < (curChi2Error + chi2Thresh) || forceOldLandmark) && chi2Error< minChi2Error){
                if (debug == 1) {
                    System.out.println("Found landmark to assoicate with");
                    System.out.println("Old chi2 Error : " + curChi2Error + " New chi2 error " + chi2Error);
                }
                returnVal = ID;
                minChi2Error = chi2Error;
            }

            //    currentPos.forgetLandmark(new Integer(nodeIndex));
            currentPos.forgetMostRecentLandmark();
        }

        /*if we havent changed the returnVal, it means we're creating a new landmark id*/
        if(returnVal == nextLandmarkId){
            if (debug == 1) {
                System.out.println("Creating new landmark with ID " + returnVal + " actual id is " + det.id);
            }
            if(mapMyIdToRealId.containsValue(new Integer(det.id))){
                numErrors++;
            }
            mapMyIdToRealId.put(new Integer(returnVal), new Integer(det.id)); 
            System.out.println("Adding " + returnVal + " " + det.id);
            nextLandmarkId++;
        }
        else{/*found old landmark*/
            if (debug == 1) {
            System.out.println("using old id " + returnVal);
            }
            if(mapMyIdToRealId.get(returnVal) != det.id){
                numErrors++;
            }

        }


        return returnVal;
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
        numEdgesToAddToSigma++;

        nextJacobRowIndex += lastOdNode.stateLength();

        /*adds relative readings*/
        allObservations.add(new OdNode(nodeIndex, nextAbsStateRowIndex, x, y, t));

        nextAbsStateRowIndex += odNode.stateLength();
        /*save last node to get next global position*/
        lastOdNode = odNode;

        for(Simulator.landmark_t det: dets){
            int thisLandmarkId;
            if(landmarksHaveId){
                thisLandmarkId = det.id;
            }
            else{
                thisLandmarkId = getLandmarkId(odNode, det, odom, lastOdNode);
               
            }

            /*If this is a duplicate*/
            if(landmarksSeen.containsKey(thisLandmarkId)){

                /*Same index as the one we already found*/
                nodeIndex = landmarksSeen.get(thisLandmarkId);

                /*add the observation, but not to the state vector*/
                /*NOTE i dont think the abs number matters for observations*/
                Node landNode = new LandNode(nodeIndex,  stateVector.get(nodeIndex).getAbsIndex(), det.obs[0], det.obs[1], thisLandmarkId);
                allObservations.add(landNode);

                Edge landEdge = new LandEdge(nextJacobRowIndex, lastOdNode.getAbsIndex(), stateVector.get(nodeIndex).getAbsIndex(), lastOdNode, landNode);

                // Hopefully the odom from a land edge is never used!
                landEdge.setOdom(odom);
                allEdges.add(landEdge);
                numEdgesToAddToSigma++;

                nextJacobRowIndex += landNode.stateLength();
                odNode.sawLandmark(nodeIndex);

            }
            else{
                /*add the landmark to the state vector and note we saw it, use global coords*/

                nodeIndex = stateVector.size();
                double []pos = LandUtil.rThetaToXY(det.obs[0], det.obs[1], newPos[0], newPos[1], newPos[2]);

                Node landNode = new LandNode(nodeIndex, nextAbsStateRowIndex, pos[0], pos[1], thisLandmarkId);
                odNode.sawLandmark(nodeIndex);
                Edge landEdge = new LandEdge(nextJacobRowIndex, lastOdNode.getAbsIndex(), nextAbsStateRowIndex, lastOdNode, landNode);
                landEdge.setOdom(odom);
                allEdges.add(landEdge);
                numEdgesToAddToSigma++;

                nextJacobRowIndex += landNode.stateLength();

                allObservations.add(new LandNode(nodeIndex, nextAbsStateRowIndex, det.obs[0], det.obs[1], thisLandmarkId));
                nextAbsStateRowIndex += landNode.stateLength();
                stateVector.add(landNode);

                landmarksSeen.put(new Integer(thisLandmarkId), new Integer(nodeIndex));




            }
            cumulativeSigmaInverse = GraphMath.addToSigmaInverse(cumulativeSigmaInverse, numEdgesToAddToSigma, allEdges, nextJacobRowIndex, pin);
            numEdgesToAddToSigma = 0;

        }//end landmarks.

        cumulativeSigmaInverse = GraphMath.addToSigmaInverse(cumulativeSigmaInverse, numEdgesToAddToSigma, allEdges, nextJacobRowIndex, pin);
        numEdgesToAddToSigma = 0;

        if (debug == 1) {
            System.out.println("BEGINNING LEAST SQUARES");
        }

        for(int i = 0; i < numConverge; i++){
            covList.clear();


            jacobList = GraphMath.getJacobList(allEdges, stateVector);
            Matrix J = JacobBlock.assemble(nextJacobRowIndex, nextAbsStateRowIndex, jacobList, numPinnedRows, pinningJacob, pin);

            // Matrix deltaX  = GraphMath.calcDeltaX(J, cumulativeSigmaInverse, allObservations, lambda, pin, stateVector);

            Matrix deltaX  = GraphMath.calcFASTDeltaX(allEdges, allObservations, lambda, pin, stateVector, nextAbsStateRowIndex, pinningJacob, pinningCov);


            if(debug == 1 || !landmarksHaveId){
                double [] residuals = GraphMath.getResiduals(stateVector, allObservations, pin);
                curChi2Error        = getChi2Error(J, deltaX, residuals, cumulativeSigmaInverse);
            }

            //Show Chi2 error
            if (debug == 1) {
                System.out.println(curChi2Error);
            }

            int index = 0;

            for (Node node : stateVector){
                index += node.addToState(deltaX.copyAsVector(), index);
            }

            if (numUpdates == 383 && i == numConverge -1) {
                long endTime = System.nanoTime();
                double elapsedTime = (endTime-startTime)/1000000000f;
                System.out.printf("Simulation time: %.2f\n", elapsedTime);
                try{
                FileWriter outFile = new FileWriter("data.txt");
                PrintWriter out = new PrintWriter(outFile);
                if(landmarksHaveId){

                    out.println(numConverge + " " + curChi2Error);  
                }
                else{
                    out.println("" + numErrors);
                }

                out.close();
                }
                catch (Exception e){
                    System.out.println("Caught exception for writing to file");
                    assert(false);
                }
            }

        } // End all iterations of Ax = b

        drawFrame();

        drawDummy(dets);
        if (debug == 1) {
            System.out.println("We have seen "+ landmarksSeen.size() + " landmarks and the state vector is " + nextAbsStateRowIndex+  " long");
        }
        System.out.println("number of errors: " + numErrors);
    }
    private void drawFrame(){
        // Grab our best guesses of the robot path and landmark positions from our state vector
        trajectory.clear();
        estimatedLandmarkPositions.clear();
        for (Node node : stateVector) {
            if (node.isLand()) {
                double[] state = node.getState();
                estimatedLandmarkPositions.add(new double[] {state[0], state[1]});
            } else {
                double[] state = node.getState();
                trajectory.add(new double[] {state[0], state[1]});
                xyt = state;
            }
        }
    }

    public double getChi2Error(Matrix J, Matrix deltaX, double[] resid, Matrix sigmaInv) {

        double chi2Error     = 0;
        double DOF           = J.getRowDimension() - deltaX.getRowDimension();
        Matrix jDeltaX       = J.times(deltaX);
        Matrix jDeltaXMinusR = jDeltaX.minus(Matrix.columnMatrix(resid));
        chi2Error            = jDeltaXMinusR.transpose().times(sigmaInv).times(jDeltaXMinusR).get(0,0);
        chi2Error            = chi2Error / DOF;
        return chi2Error;

    }

    public void drawDummy(ArrayList<Simulator.landmark_t> landmarks)
    {
        // Draw local Trajectory -- the red robot path -- our least squares "best guess"
        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-local");
            vb.addBack(new VisLines(new VisVertexData(trajectory),
                                    new VisConstantColor(new Color(160,30,30)),
                                    1.5, VisLines.TYPE.LINE_STRIP));
            vb.swap();
        }


        // Draw our least squares best guess of the landmark positions
        {
            VisWorld.Buffer vb = vw.getBuffer("landmark-position-local");
            vb.addBack(new VisPoints(new VisVertexData(estimatedLandmarkPositions),
                                     new VisConstantColor(new Color(255,0,0)),
                                     10.0));
            vb.swap();

        }


        ArrayList<double[]> rpoints = new ArrayList<double[]>();
        rpoints.add(new double[]{-.3, .3});
        rpoints.add(new double[]{-.3, -.3});
        rpoints.add(new double[]{.45,0});

        // Draws the robot triangle at xyt
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

        // Draw the landmark observations -- the blue lines shooting out of the red bot
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
