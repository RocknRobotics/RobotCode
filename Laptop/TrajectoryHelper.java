package frc.robot.Laptop;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RoboRio.Constants;

public class TrajectoryHelper {
    @SuppressWarnings("unused")
    private NetworkTableInstance myInstance;

    //The points to use when calculating possible waypoints
    //First is the x step value associated with it (length x steps)
    //Second if the above but for y steps
    //Third is length two and contains the x value then y value in metres associated with that point
    private double masterpoints[][][];
    //The current Trajectory to follow
    private Trajectory myTrajectory;
    //The time that the Trajectory started at
    private double trajectoryStartTime;
    //Controller for following the Trajectory
    private HolonomicDriveController myHolonomicDriveController;

    //Need to have copies essentially in order to change them if the alliance colour is red
    private double outXMin[];
    private double outYMin[];
    private double outXMax[];
    private double outYMax[];

    public TrajectoryHelper(NetworkTableInstance inst) {
        myInstance = inst;
    }

    public void create() {
        //If we're red alliance
        if(DriverStation.getAlliance() != DriverStation.Alliance.Blue) {
            //Flip em
            for(int i = 0; i < Constants.TrajectoryConstants.outXMin.length; i++) {
                outXMin[i] = Constants.TrajectoryConstants.fieldLength - Constants.TrajectoryConstants.outXMin[i];
                outXMax[i] = Constants.TrajectoryConstants.fieldLength - Constants.TrajectoryConstants.outXMax[i];
                outYMin[i] = Constants.TrajectoryConstants.fieldWidth - Constants.TrajectoryConstants.outYMin[i];
                outYMax[i] = Constants.TrajectoryConstants.fieldWidth - Constants.TrajectoryConstants.outYMax[i];
            }
        } else {
            //Keep it as is
            outXMin = Constants.TrajectoryConstants.outXMin;
            outYMin = Constants.TrajectoryConstants.outYMin;
            outXMax = Constants.TrajectoryConstants.outXMax;
            outYMax = Constants.TrajectoryConstants.outYMax;
        }

        for(int r = 0; r < masterpoints.length; r++) {
            for(int c = 0; c < masterpoints.length; c++) {
                //The +2 and +1 make it so that the ends aren't included when calculating preset points (since they would never work)
                masterpoints[r][c][0] = (Constants.TrajectoryConstants.fieldLength / (Constants.TrajectoryConstants.xSteps + 2)) * (r + 1);
                masterpoints[r][c][1] = (Constants.TrajectoryConstants.fieldWidth / (Constants.TrajectoryConstants.ySteps + 2)) * (c + 1);
            }
        }

        ProfiledPIDController temp = new ProfiledPIDController(Constants.TrajectoryConstants.kAngle[0], Constants.TrajectoryConstants.kAngle[1], Constants.TrajectoryConstants.kAngle[2], Constants.TrajectoryConstants.angleConstraints);
        temp.enableContinuousInput(-Math.PI, Math.PI);

        myHolonomicDriveController = new HolonomicDriveController(new PIDController(Constants.TrajectoryConstants.kX[0], Constants.TrajectoryConstants.kX[1], Constants.TrajectoryConstants.kX[2]), 
        new PIDController(Constants.TrajectoryConstants.kY[0], Constants.TrajectoryConstants.kY[1], Constants.TrajectoryConstants.kY[2]), temp);
        myHolonomicDriveController.setTolerance(Constants.TrajectoryConstants.controllerTolerance);
    }

    public void close() {
        masterpoints = null;
        myTrajectory = null;
        myHolonomicDriveController.getXController().close();
        myHolonomicDriveController.getYController().close();
        myHolonomicDriveController = null;
    }

    //Returns if the robot is within the tolerance specified by the controllerTolerance
    public boolean atReference() {
        return myHolonomicDriveController.atReference();
    }

    //Both the ChassissSpeeds methods are FIELD-RELATIVE
    public ChassisSpeeds getChassisSpeeds(Pose2d currentPose, double currTime, Rotation2d targetHeading) {
        return myHolonomicDriveController.calculate(currentPose, getState(currTime), targetHeading);
    }

    public ChassisSpeeds getChassisSpeeds(Pose2d currentPose, Trajectory.State targetState, Rotation2d targetHeading) {
        return myHolonomicDriveController.calculate(currentPose, targetState, targetHeading);
    }

    public void setTrajectory(Trajectory aTrajectory) {
        myTrajectory = aTrajectory;
        trajectoryStartTime = -1d;
    }

    //Returns the State at a given time, assumes Trajectory is not null
    public Trajectory.State getState(double currTime) {
        //Record the current time as the start time if we haven't "started" this trajectory yet
        if(trajectoryStartTime == -1d) {
            trajectoryStartTime = currTime;
        }

        //Trajectory class automatically handles t = 0 or t > total time, so we can safely call this in all cases
        return myTrajectory.sample(currTime - trajectoryStartTime);
    }

    //Full Trajectory control aside from the config kinematics and max speed/acceleration, but at that point just call the actual method
    public Trajectory createTrajectory(Pose2d currentPose, List<Translation2d> middlePoints, Pose2d endPose, boolean reverse, double startVelocity, double endVelocity, List<TrajectoryConstraint> constraints) {
        return TrajectoryGenerator.generateTrajectory(currentPose, middlePoints, endPose, createTrajectoryConfig(startVelocity, endVelocity, constraints, reverse));
    }

    //Assumes start and end velocity are 0
    //This here is the one we'll probably use?
    public Trajectory createTrajectory(Pose2d currentPose, Pose2d endPose, boolean reverse) {
        return TrajectoryGenerator.generateTrajectory(currentPose, createWaypoints(currentPose, endPose), endPose, createTrajectoryConfig(0d, 0d, null, reverse));
    }

    //Easier config creation
    public TrajectoryConfig createTrajectoryConfig(double startVelocity, double endVelocity, List<TrajectoryConstraint> constraints, boolean reverse) {
        TrajectoryConfig output = new TrajectoryConfig(Constants.maxTranslationalSpeed, Constants.TrajectoryConstants.maxTranslationalAcceleration);
        output.setKinematics(Constants.talonConstants.driveConstants.driveTalonKinematics);
        output.setStartVelocity(startVelocity);
        output.setEndVelocity(endVelocity);
        output.addConstraints(constraints);
        output.setReversed(reverse);

        return output;
    }

    //Not super good algorithm, but should get the job done. Finds waypoints given current/end pose and the field boundaries
    public List<Translation2d> createWaypoints(Pose2d currentPose, Pose2d endPose) {
        //I should probably comment all this huh
        //The Translation2ds to be returned
        List<Translation2d> output = new ArrayList<Translation2d>();
        //The indexes for which Nodes have already been created for (you'll see in a momement what I mean)
        boolean deadIndexes[][] = new boolean[masterpoints.length][masterpoints[0].length];
        //Let me preface by saying that every time index is referred to in this method it is in relation to the masterpoints array
        //(I'm pretty sure anyways. Includes the above variable). This gives the index numbers that most closely match the current Pose
        int[] currIndexes = currentIndexes(currentPose, endPose);

        //A node for the tree class (tree class is below node)
        class Node {
            //The total distance it takes to get to this Node from the starting position
            double totDistance;
            //The list of nodes branching off this node
            List<Node> myNext;
            //The node this node branches off of
            Node parent;
            //The xIndex (in the masterpoints array) of the point this nodes represents
            int xIndex;
            //The yIndex (in the masterpoints array) of the point this nodes represents
            int yIndex;

            //Self-explanatory
            Node(int xIndex, int yIndex, Node parent, double totDistance) {
                this.xIndex = xIndex;
                this.yIndex = yIndex;
                this.totDistance = totDistance;
                this.parent = parent;
            }

            //Calculates the myNext list (useBounds describes whether or not we should bound the x and y values when we test which nodes to branch off of this one)
            void calcNext(boolean useBounds) {
                //If this node's associated x value is less than the endPose x value then this is true
                boolean lessX = masterpoints[xIndex][yIndex][0] <= endPose.getX();
                //Above but for y
                boolean lessY = masterpoints[xIndex][yIndex][1] <= endPose.getX();

                //If we're bounding it
                if(useBounds) {
                    //If my x is less than end x
                    if(lessX) {
                        //De-increment until the new x value is less than the end x value
                        for(int r = xIndex; masterpoints[r][0][0] >= endPose.getX(); r--) {
                            //If my y is less than end y
                            if(lessY) {
                                //De-increment
                                for(int c = yIndex; masterpoints[0][c][1] >= endPose.getY(); c--) {
                                    //If we've have NOT already created a node with these x/y indexes
                                    if(!deadIndexes[r][c]) {
                                        //Check if it touches a zone from my current position to this new node
                                        if(!touchesZone(masterpoints[r][c])) {
                                            //If it works, then add it
                                            myNext.add(new Node(r, c, this, totDistance + distance(r, c)));
                                        }
                                    }
                                }
                            } else {
                                //Increment
                                for(int c = yIndex; masterpoints[0][c][1] <= endPose.getY(); c++) {
                                    //Check if not dead
                                    if(!deadIndexes[r][c]) {
                                        //If doesn't touch a zone
                                        if(!touchesZone(masterpoints[r][c])) {
                                            //Add
                                            myNext.add(new Node(r, c, this, totDistance + distance(r, c)));
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        //Increment row
                        for(int r = xIndex; masterpoints[r][0][0] <= endPose.getX(); r++) {
                            if(lessY) {
                                //De-increment column
                                for(int c = yIndex - 1; masterpoints[0][c][1] >= endPose.getY(); c--) {
                                    //Dead?
                                    if(!deadIndexes[r][c]) {
                                        //Touches?
                                        if(!touchesZone(masterpoints[r][c])) {
                                            //Add :)
                                            myNext.add(new Node(r, c, this, totDistance + distance(r, c)));
                                        }
                                    }
                                }
                            } else {
                                //Increment column
                                for(int c = yIndex; masterpoints[0][c][1] <= endPose.getY(); c++) {
                                    //Zombie?
                                    if(!deadIndexes[r][c]) {
                                        //Committing assault?
                                        if(!touchesZone(masterpoints[r][c])) {
                                            //Add :)
                                            myNext.add(new Node(r, c, this, totDistance + distance(r, c)));
                                        }
                                    }
                                }
                            }
                        }
                    }
                } else {
                    //Repeat of above except we make the row and column span the entire 0--masterpoints length (ie x/y Steps)
                    if(lessX) {
                        for(int r = masterpoints.length; r >= 0; r--) {
                            if(lessY) {
                                for(int c = masterpoints[r].length; c >= 0; c--) {
                                    if(!deadIndexes[r][c]) {
                                        if(!touchesZone(masterpoints[r][c])) {
                                            myNext.add(new Node(r, c, this, totDistance + distance(r, c)));
                                        }
                                    }
                                }
                            } else {
                                for(int c = 0; c < masterpoints[r].length; c++) {
                                    if(!deadIndexes[r][c]) {
                                        if(!touchesZone(masterpoints[r][c])) {
                                            myNext.add(new Node(r, c, this, totDistance + distance(r, c)));
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        for(int r = 0; r < masterpoints.length; r++) {
                            if(lessY) {
                                for(int c = masterpoints[r].length; c >= 0; c--) {
                                    if(!deadIndexes[r][c]) {
                                        if(!touchesZone(masterpoints[r][c])) {
                                            myNext.add(new Node(r, c, this, totDistance + distance(r, c)));
                                        }
                                    }
                                }
                            } else {
                                for(int c = 0; c < masterpoints[r].length; c++) {
                                    if(!deadIndexes[r][c]) {
                                        if(!touchesZone(masterpoints[r][c])) {
                                            myNext.add(new Node(r, c, this, totDistance + distance(r, c)));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            //Helper for determining if any zone between this node's points and the given points will be crossed (since we don't want
            //to create a path that crosses out of bounds zones)
            boolean touchesZone(double nextPoints[]) {
                double currPoints[] = masterpoints[xIndex][yIndex];

                for(int i = 0; i < outXMin.length; i++) {
                    //Assumes t == 1 when currPoints reaches nextPoints
                    //Therefore tMin and tMax should be less than 1
                    //Arbitrary I guess? Prevents division by 0 or a really small number
                    if(Math.abs(nextPoints[0] - currPoints[0]) > 0.0001) {
                        double tMin = (outXMin[i] - currPoints[0]) / (nextPoints[0] - currPoints[0]);
                        double tMax = (outXMax[i] - currPoints[0]) / (nextPoints[0] - currPoints[0]);
                        //The y values at the given point
                        double yMin = tMin * (nextPoints[1] - currPoints[1]); //Enter
                        double yMax = tMax * (nextPoints[1] - currPoints[1]); //Exit
    
                        //Basically if tMin is not negative and yMin would be in the OB zone or if that happens with tMax, then it touched a zone
                        if((tMin >= 0 && yMin - Constants.TrajectoryConstants.robotAvoidance >= outYMin[i] && yMin + Constants.TrajectoryConstants.robotAvoidance <= outYMax[i]) || 
                        (tMax >= 0 && yMax - Constants.TrajectoryConstants.robotAvoidance >= outYMin[i] && yMax + Constants.TrajectoryConstants.robotAvoidance <= outYMax[i])) {
                            return true;
                        }
                    }

                    //Repeat of the above but using y as the basis for the t variable (necessary because vertical lines exist and wouldn't be caught by the above)
                    if(Math.abs(nextPoints[1] - currPoints[1]) > 0.0001) {
                        double tMin = (outYMin[i] - currPoints[1]) / (nextPoints[1] - currPoints[1]);
                        double tMax = (outYMax[i] - currPoints[1]) / (nextPoints[1] - currPoints[1]);
                        //The x values at the given point
                        double xMin = tMin * (nextPoints[0] - currPoints[0]);
                        double xMax = tMax * (nextPoints[0] - currPoints[0]);
    
                        //Basically if tMin is not negative and xMin would be in the OB zone or if that happens with tMax, then it touched a zone
                        if((tMin >= 0 && xMin - Constants.TrajectoryConstants.robotAvoidance >= outXMin[i] && xMin + Constants.TrajectoryConstants.robotAvoidance <= outXMax[i]) || 
                        (tMax >= 0 && xMax - Constants.TrajectoryConstants.robotAvoidance >= outXMin[i] && xMax + Constants.TrajectoryConstants.robotAvoidance <= outXMax[i])) {
                            return true;
                        }
                    }
                }

                return false;
            }

            //Helper function for distance
            double distance(int r, int c) {
                return Math.sqrt(Math.pow(masterpoints[r][c][0] - masterpoints[xIndex][yIndex][0], 2) + Math.pow(masterpoints[r][c][1] - masterpoints[xIndex][yIndex][1], 2));
            }
        }

        //This one is even more fun to comment
        class Tree {
            //Stores a map of size x/y Steps (again, the same as masterpoints lengths) that stores a singular Node at each point
            //Used for easier lookup of nodes with a given x/y Index
            Node masterMap[][];
            //Returns whether a Node from this x/y Index to the endPose (given in constructor) would cross any zones
            //Decreases number of Nodes checked in the best() method
            boolean wouldWork[][];
            //The list of lowest nodes (ie the ones who aren't parents yet). This way we don't do repeat calcNext()s on the same node
            List<Node> lowest;
            //The Node with the lowest total distance whose indexes are true in the wouldWork array
            Node best;
            //The x/y points for the head node
            double head[];

            //Constructor
            Tree(double[] head, Pose2d endPose) {
                masterMap = new Node[Constants.TrajectoryConstants.xSteps][Constants.TrajectoryConstants.ySteps];
                wouldWork = wouldWorkInit(endPose);
                lowest = new ArrayList<Node>();
                this.head = head;
            }

            //Populates the wouldWork array with whether or not each point could get to the endPose without problems
            boolean[][] wouldWorkInit(Pose2d endPose) {
                boolean output[][] = new boolean[Constants.TrajectoryConstants.xSteps][Constants.TrajectoryConstants.ySteps];

                for(int r = 0; r < output.length; r++) {
                    for(int c = 0; c < output[r].length; c++) {
                        output[r][c] = !touchesZoneStripped(masterpoints[r][c], endPose);
                    }
                }

                return output;
            }

            //This way we don't have to create a new Node in order to access the touchesZone method
            boolean touchesZoneStripped(double[] currPoints, Pose2d endPose) {
                for(int i = 0; i < outXMin.length; i++) {
                    //Assumes t == 1 when currPoints reaches endPose
                    //Therefore tMin and tMax should be less than 1
                    //Arbitrary I guess? Prevents division by 0 or a really small number
                    if(Math.abs(endPose.getX() - currPoints[0]) > 0.0001) {
                        double tMin = (outXMin[i] - currPoints[0]) / (endPose.getX() - currPoints[0]);
                        double tMax = (outXMax[i] - currPoints[0]) / (endPose.getX() - currPoints[0]);
                        //The y values at the given point
                        double yMin = tMin * (endPose.getX() - currPoints[1]);
                        double yMax = tMax * (endPose.getX() - currPoints[1]);
    
                        //Basically if tMin is not negative and yMin would be in the OB zone or if that happens with tMax, then it touched a zone
                        if((tMin >= 0 && yMin - Constants.TrajectoryConstants.robotAvoidance >= outYMin[i] && yMin + Constants.TrajectoryConstants.robotAvoidance <= outYMax[i]) || 
                        (tMax >= 0 && yMax - Constants.TrajectoryConstants.robotAvoidance >= outYMin[i] && yMax + Constants.TrajectoryConstants.robotAvoidance <= outYMax[i])) {
                            return true;
                        }
                    }

                    //Repeat of the above but using y as the basis for the t variable (necessary because vertical lines exist and wouldn't be caught by the above)
                    if(Math.abs(endPose.getY() - currPoints[1]) > 0.0001) {
                        double tMin = (outYMin[i] - currPoints[1]) / (endPose.getY() - currPoints[1]);
                        double tMax = (outYMax[i] - currPoints[1]) / (endPose.getY() - currPoints[1]);
                        //The x values at the given point
                        double xMin = tMin * (endPose.getY() - currPoints[0]);
                        double xMax = tMax * (endPose.getY() - currPoints[0]);
    
                        //Basically if tMin is not negative and xMin would be in the OB zone or if that happens with tMax, then it touched a zone
                        if((tMin >= 0 && xMin - Constants.TrajectoryConstants.robotAvoidance >= outXMin[i] && xMin + Constants.TrajectoryConstants.robotAvoidance <= outXMax[i]) || 
                        (tMax >= 0 && xMax - Constants.TrajectoryConstants.robotAvoidance >= outXMin[i] && xMax + Constants.TrajectoryConstants.robotAvoidance <= outXMax[i])) {
                            return true;
                        }
                    }
                }

                return false;
            }

            //It used to be way worse before I made the masterMap
            void calcNext() {
                //What we'll set lowest to after calc the next for every node in lowest
                List<Node> newLowest = new ArrayList<Node>();

                //Special case for head---assumes that lowest will always be populated otherwise
                if(lowest.size() == 0) {
                    //Node representation of head using the calculated indexes earlier so we can call calcNext()
                    //Don't need to set the total distance correctly since we'll fix that in a moment
                    Node tempHead = new Node(currIndexes[0], currIndexes[1], null, 0d);
                    tempHead.calcNext(Constants.TrajectoryConstants.useNodeBounds);

                    for(Node littleNode: tempHead.myNext) {
                        //Corrects the distances
                        littleNode.totDistance = Math.sqrt(Math.pow(masterpoints[littleNode.xIndex][littleNode.yIndex][0] - currentPose.getX(), 2) + Math.pow(masterpoints[littleNode.xIndex][littleNode.yIndex][1] - currentPose.getY(), 2));

                        //Gets the node in the mastermap corresponding to littleNode's indexes
                        Node mapNode = masterMap[littleNode.xIndex][littleNode.yIndex];

                        //If the node in the mastermap is null or it has a greater total distance
                        if(mapNode == null || littleNode.totDistance < mapNode.totDistance) {
                            //Add littleNode :)
                            newLowest.add(littleNode);
                            //Set masterMap to littleNode
                            masterMap[littleNode.xIndex][littleNode.yIndex] = littleNode;

                            //If the Node works for getting to the endpoint and there is no best node or the littleNode has a lower total distance than the best
                            if(wouldWork[littleNode.xIndex][littleNode.yIndex] && (best == null || littleNode.totDistance < best.totDistance)) {
                                //Make littleNode the best
                                best = littleNode;
                            }
                        }
                    } 
                } else {
                    for(Node aNode: lowest) {
                        aNode.calcNext(Constants.TrajectoryConstants.useNodeBounds);

                        for(Node littleNode: aNode.myNext) {
                            //Gets the node in the mastermap corresponding to littleNode's indexes
                            Node mapNode = masterMap[littleNode.xIndex][littleNode.yIndex];

                            //If the node in the mastermap is null or it has a greater total distance
                            if(mapNode == null || littleNode.totDistance < mapNode.totDistance) {
                                //Add littleNode :)
                                newLowest.add(littleNode);
                                //Set masterMap to littleNode
                                masterMap[littleNode.xIndex][littleNode.yIndex] = littleNode;

                                //If the Node works for getting to the endpoint and there is no best node or the littleNode has a lower total distance than the best
                                if(wouldWork[littleNode.xIndex][littleNode.yIndex] && (best == null || littleNode.totDistance < best.totDistance)) {
                                    //Make littleNode the best
                                    best = littleNode;
                                }
                            }
                        }
                    }

                    //We want to kill bounds after every Node in lowest has calcNext()ed so that we don't accidentally limit our node options while calcNext()ing
                    for(Node aNode: newLowest) {
                        deadIndexes[aNode.xIndex][aNode.yIndex] = true;
                    }
                }

                lowest = newLowest;
            }
        }
        //End nested class construction ----------------------------------------------------------------------------------
        
        //Getka naming
        Tree wow = new Tree(new double[]{currentPose.getX(), currentPose.getY()}, endPose);

        //Checks if the currentPose can get to the endPose without problems
        if(!wow.touchesZoneStripped(wow.head, endPose)) {
            //Returns empty list if it can
            return output;
        }

        //Until we have a node that works, keep calculating new layers
        while(wow.best == null) {
            wow.calcNext();
        }

        //Traverses through the best node's line starting with best
        for(Node temp = wow.best; temp != null; temp = temp.parent) {
            //Adds at start so that the list isn't reversed
            output.add(0, new Translation2d(masterpoints[temp.xIndex][temp.yIndex][0], masterpoints[temp.xIndex][temp.yIndex][1]));

            //Head node == tree.head, which uses pseudo bounds
            //Therefore, we want to check if the current pose can make it to the node below tree.head without any problems
            //Since then we can exclude tree.head
            if(temp.parent.parent == null && temp.parent != null) {
                //If from currentPose to the current Node (one below top) works, then don't add top
                if(!wow.touchesZoneStripped(masterpoints[temp.xIndex][temp.yIndex], currentPose)) {
                    break;
                }
            }
        }

        return output;
    }

    //Returns the indexes one "closer" to the endPose based on the currentPose
    public int[] currentIndexes(Pose2d currentPose, Pose2d endPose) {
        //The indexes to be returned
        int output[] = new int[2];
        boolean lessX = currentPose.getX() < endPose.getX();
        boolean lessY = currentPose.getY() < endPose.getY();

        if(lessX) {
            //If less increment, since we want to be one index "closer" to the endPose than the currentPose points
            for(int i = 0; i < masterpoints.length; i++) {
                //If we're at the end or the currentPose x fits within the current masterpoint and next masterpoint
                if(i == masterpoints.length - 1 || (masterpoints[i][0][0] <= currentPose.getX() && masterpoints[i + 1][0][0] >= currentPose.getX())) {
                    output[0] = i;
                    break;
                }
            }
        } else {
            //Above but with de-incrementation
            for(int i = masterpoints.length - 1; i >= 0; i--) {
                //If i is the start or currentPose x fits within the current masterpoint and "previous" masterpoint
                if(i == 0 || (masterpoints[i][0][0] >= currentPose.getX() && masterpoints[i - 1][0][0] <= currentPose.getX())) {
                    output[0] = i;
                    break;
                }
            }
        }

        //Above but for y
        if(lessY) {
            for(int i = 0; i < masterpoints[0].length; i++) {
                if(i == masterpoints[0].length - 1 || (masterpoints[0][i][1] <= currentPose.getY() && masterpoints[0][i + 1][1] >= currentPose.getY())) {
                    output[1] = i;
                    break;
                }
            }
        } else {
            for(int i = masterpoints[0].length - 1; i >= 0; i--) {
                if(i == 0 || (masterpoints[0][i][1] >= currentPose.getY() && masterpoints[0][i - 1][1] <= currentPose.getY())) {
                    output[1] = i;
                    break;
                }
            }
        }

        return output;
    }
}
