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

    double xyt[] = new double[3]; // dead reconning

    ArrayList<double[]> trajectory = new ArrayList<double[]>();

    ArrayList<Node> allObservations = new ArrayList<Node>();
    ArrayList<Node> stateVector = new ArrayList<Node>();

    ArrayList<JacobBlock> jacobList = new ArrayList<JacobBlock>();

    private static int numUpdates = 0;

    /*maps from the landmark id to the index in the state vector array*/
    HashMap<Integer, Integer> landmarksSeen = new HashMap<Integer, Integer>();
    double baseline;
    OdNode lastOdNode =null;


    public void init(Config _config, VisWorld _vw)
    {
        config  = _config;
        vw = _vw;
	Edge.config = _config;
        baseline = config.requireDouble("robot.baseline_m");
	lastOdNode = new OdNode(0, 0, 0, 0);
	stateVector.add(lastOdNode);
    }


    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets)
    {

        numUpdates++;
	DenseVec ticksXYT = TicksUtil.ticksToXYT(odom, baseline);

        double x = ticksXYT.get(0);
        double y = ticksXYT.get(1);
        double t = ticksXYT.get(2);;
        int index = stateVector.size();

	
	double [] newPos = LinAlg.xytMultiply(lastOdNode.getState(), new double[]{x, y, t});

	/*adds global coords*/
	OdNode odNode = new OdNode(index, newPos[0], newPos[1], newPos[2]);      
	stateVector.add(odNode);
	
	/*adds relative readings*/
	allObservations.add(new OdNode(index, x, y, t));

	/*save last node to get next global position*/
	lastOdNode = odNode;


        for(Simulator.landmark_t det: dets){
	    
	    /*If this is a duplicate*/             
	    if(landmarksSeen.containsKey(det.id)){
		
		/*Same index as the one we already found*/
		index = landmarksSeen.get(det.id);

		/*add the observation, but not to the state vector*/
		Node landNode = new LandNode(index, det.obs[0], det.obs[1], det.id);
		allObservations.add(landNode);

		continue;
            }

	    /*add the landmark to the state vector and note we saw it, use global coords*/
	    index = stateVector.size();
	    double []pos = LandUtil.rThetaToXY(det.obs[0], det.obs[1], newPos[0], newPos[1], newPos[2]);
	    Node landNode = new LandNode(index, pos[0], pos[1], det.id);   
            stateVector.add(landNode);
           
	    landmarksSeen.put(new Integer(det.id), new Integer(index));
	    
        }

	Matrix J = JacobBlock.assemble(allObservations.size(), stateVector.size(), jacobList);

        System.out.println("********State vector*********");
        for(Node n : stateVector){
            System.out.println(n);
	    
        }
	

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
