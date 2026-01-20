package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;

public class RobotSecondary extends Brain {
    private enum Role {UNDEFINED, EXPLORER_ALPHA, EXPLORER_BETA}
    private enum State {
        MOVE,
        TURNING_NORTH,
        TURNING_SOUTH,
        TURNING_WEST,
        TURNING_EAST,
        TURNING_BACK,
        GOING_NORTH,
        CHECKING_WEST,
        CHECKING_EAST,
        CHECKING_SOUTH,
        EXPLORATION_COMPLETE,
        IDLE
    }

    private Role role = Role.UNDEFINED;
    private State state = State.IDLE;
    private String robotName = "undefined";
    private double myX, myY;
    private boolean isMoving = false;
    private static final double ANGLE_PRECISION = 0.1;
    private double targetAngle;
    private int stepCounter = 0;

    // Map bounds discovered
    private double northBound = -1;
    private double westBound = -1;
    private double eastBound = -1;
    private double southBound = -1;

    private static final double TEAMMATE_DETECTION_DISTANCE = 10000;
    private static final double ENEMY_DETECTION_DISTANCE = 200;
    private static final double WRECK_DETECTION_DISTANCE = 100;
    private static final double SECONDARY_BOT_DETECTION_DISTANCE = 100;

    @Override
    public void activate() {
        /*
        Alpha is the one on bottom - explores north (top)
        Beta is the one on top - explores south (bottom)
         */
        int botsAbove = 0;
        int botsBelow = 0;

        for (IRadarResult o : detectRadar()){
            if(o.getObjectType() == IRadarResult.Types.TeamSecondaryBot){
                double relativeAngle = normalize(o.getObjectDirection() - myGetHeading());

                // Check if teammate is above (north direction range: π/4 to 3π/4)
                if (relativeAngle > Math.PI / 4 && relativeAngle < 3 * Math.PI / 4) {
                    botsAbove++;
                }
                // Check if teammate is below (south direction range: 5π/4 to 7π/4)
                else if (relativeAngle > 5 * Math.PI / 4 && relativeAngle < 7 * Math.PI / 4) {
                    botsBelow++;
                }
            }
        }

        // Determine initial position from Parameters
        myX = Parameters.teamASecondaryBot1InitX;
        myY = Parameters.teamASecondaryBot1InitY;

        if (botsAbove == 0 && botsBelow == 1) {
            // Bottom position
            role = Role.EXPLORER_ALPHA;
            robotName = "Explorer Alpha";
            System.out.println("I am " + robotName + " (bottom), I will explore the NORTH area.");
            state = State.TURNING_SOUTH; // Start idle, waiting for further implementation
            targetAngle = Parameters.SOUTH;
        } else {
            // Top position
            role = Role.EXPLORER_BETA;
            robotName = "Explorer Beta";
            myX = Parameters.teamASecondaryBot2InitX;
            myY = Parameters.teamASecondaryBot2InitY;
            System.out.println("I am " + robotName + " (top), I will explore the SOUTH area.");
            state = State.TURNING_NORTH;
            targetAngle = Parameters.NORTH;
        }

        isMoving = false;
    }

    @Override
    public void step() {
        updateOdometry();
        readTeammateMessages();
        switch (state){
            case TURNING_NORTH:
                if (isSameDirection(myGetHeading(), Parameters.NORTH)) {
                    state = State.GOING_NORTH;
                } else {
                    Parameters.Direction dir = getTurnDirection(myGetHeading(), Parameters.NORTH);
                    stepTurn(dir);
                }
                break;

            case GOING_NORTH:
                if (detectWall()) {
                    // Reached north wall!
                    northBound = myY;
                    broadcastBorders("NORTH");
                    state = State.TURNING_WEST;
                    targetAngle = Parameters.WEST;
                } else {
                    myMove();
                }
                break;
            case TURNING_SOUTH:
                if (isSameDirection(myGetHeading(), Parameters.SOUTH)) {
                    state = State.CHECKING_SOUTH;
                } else {
                    Parameters.Direction dir = getTurnDirection(myGetHeading(), Parameters.SOUTH);
                    stepTurn(dir);
                }
                break;
            case CHECKING_SOUTH:
                if (detectWall()) {
                    // Reached south wall!
                    southBound = myY;
                    broadcastBorders("SOUTH");
                    state = State.TURNING_WEST;
                    targetAngle = Parameters.WEST;
                } else {
                    myMove();
                }
                break;
            case TURNING_WEST:
                if (isSameDirection(myGetHeading(), Parameters.WEST)) {
                    state = State.CHECKING_WEST;
                } else {
                    Parameters.Direction dir = getTurnDirection(myGetHeading(), Parameters.WEST);
                    stepTurn(dir);
                }
                break;

            case CHECKING_WEST:
                if (detectWall()) {
                    // Reached west wall!
                    westBound = myX;
                    broadcastBorders("WEST");
                    state = State.TURNING_EAST;
                    targetAngle = Parameters.EAST;
                } else {
                    myMove();
                }
                break;

            case TURNING_EAST:
                if (isSameDirection(myGetHeading(), Parameters.EAST)) {
                    state = State.CHECKING_EAST;
                } else {
                    Parameters.Direction dir = getTurnDirection(myGetHeading(), Parameters.EAST);
                    stepTurn(dir);
                }
                break;

            case CHECKING_EAST:
                if (detectWall()) {
                    // Reached east wall!
                    eastBound = myX;
                    broadcastBorders("EAST");
                    state = State.EXPLORATION_COMPLETE;
                } else {
                    myMove();
                }
                break;

            case EXPLORATION_COMPLETE:
                // TODO: Add behavior after exploration
                // For now, just stay idle
                break;

            case IDLE:
                // Beta bot waiting for implementation
                break;
        }
    }

    private Parameters.Direction getTurnDirection(double current, double target) {
        double diff = normalize(target - current);
        return (diff <= Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
    }

    private void myMove() {
        isMoving = true;
        move();
    }

    private void updateOdometry() {
        boolean blockedByWall = detectWall();
        boolean blockedByWreck = isBlockedByWreckObstacle();
        boolean blockedByTeamMate = isBlockedByTeamMate();
        boolean blockedByOpponent = isBlockedByOpponent();

        if (isMoving) {
            if (!blockedByWall && !blockedByWreck && !blockedByTeamMate && !blockedByOpponent) {
                myX += Parameters.teamASecondaryBotSpeed * Math.cos(myGetHeading());
                myY += Parameters.teamASecondaryBotSpeed * Math.sin(myGetHeading());
            }
            isMoving = false;
        }
    }

    private boolean isSameDirection(double dir1, double dir2) {
        return Math.abs(normalize(dir1) - normalize(dir2)) < ANGLE_PRECISION;
    }

    private double myGetHeading() {
        double result = getHeading();
        while (result < 0) result += 2 * Math.PI;
        while (result > 2 * Math.PI) result -= 2 * Math.PI;
        return result;
    }

    private double normalize(double dir) {
        double res = dir;
        while (res < 0) res += 2 * Math.PI;
        while (res >= 2 * Math.PI) res -= 2 * Math.PI;
        return res;
    }
    /*
    BROADCCAST FUNCTIONS
     */
    private void broadcastBorders(String border){
        String msg = "";
        switch(border) {
            case "NORTH":
                msg = "BORDER|NORTH|" + (int) northBound;
                broadcast(msg);
                break;
            case "SOUTH":
                msg = "BORDER|SOUTH|" + (int) southBound;
                broadcast(msg);
                break;
            case "EAST":
                msg = "BORDER|EAST|" + (int) eastBound;
                broadcast(msg);
                break;
            case "WEST":
                msg = "BORDER|WEST|" + (int) westBound;
                broadcast(msg);
                break;
            default:
                return;
        }
    }
    private void readTeammateMessages() {
        ArrayList<String> messages = fetchAllMessages();
        for (String msg : messages) {
            if (msg.startsWith("BORDER")) {
                try {
                    String[] parts = msg.split("\\|");
                    if (parts.length == 3) {
                        String borderType = parts[1];
                        String position = parts[2];
                        Integer pos = Integer.parseInt(position);
                        sendLogMessage(robotName + " received border info: " + borderType + " at " + pos);
                        switch (borderType) {
                            case "NORTH":
                                northBound = pos;
                                break;
                            case "SOUTH":
                                southBound = pos;
                                break;
                            case "WEST":
                                westBound = pos;
                                break;
                            case "EAST":
                                eastBound = pos;
                                break;
                        }
                    }
                } catch (Exception e) {
                    // Invalid message, ignore
                }
            }

        }
    }
    /*
    DETECTION FUNCTIONS
     */

    private boolean detectWall() {
        return detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
    }

    private boolean isInFront(double objDir) {
        double relativeAngle = normalize(objDir - myGetHeading());
        return relativeAngle < Math.PI / 4 || relativeAngle > (2 * Math.PI - Math.PI / 4);
    }

    private boolean isBehind(double objDir) {
        double relativeAngle = normalize(objDir - myGetHeading());
        return relativeAngle > (3 * Math.PI / 4) && relativeAngle < (5 * Math.PI / 4);
    }

    /*
    Obstacle functions
     */

    private boolean obstacleCheck() {
        return detectWall() || isBlockedByWreckObstacle() || isBlockedByTeamMate() || isBlockedByOpponent();
    }

    private boolean isBlockedByTeamMainBotOnly() {
        return isBlockedByTeamMainBot() && !detectWall() && !isBlockedByWreckObstacle()
                && !isBlockedBySecondaryBot() && !isBlockedByOpponent();
    }

    private boolean isBlockedBySecondaryBotOnly() {
        return isBlockedBySecondaryBot() && !detectWall() && !isBlockedByWreckObstacle()
                && !isBlockedByTeamMainBot() && !isBlockedByOpponent();
    }

    private boolean isBlockedByTeamMate() {
        return isBlockedByTeamMainBot() || isBlockedBySecondaryBot();
    }

    private boolean isBlockedBySecondaryBot() {
        for (IRadarResult o : detectRadar())
            if ((o.getObjectType() == IRadarResult.Types.TeamSecondaryBot
                    || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot)
                    && isInFront(o.getObjectDirection())
                    && !isBehind(o.getObjectDirection())
                    && o.getObjectDistance() < SECONDARY_BOT_DETECTION_DISTANCE) {
                return true;
            }
        return false;
    }

    private boolean isBlockedByOpponent() {
        for (IRadarResult o : detectRadar())
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot
                    || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot)
                    && isInFront(o.getObjectDirection())
                    && !isBehind(o.getObjectDirection())
                    && o.getObjectDistance() < ENEMY_DETECTION_DISTANCE) {
                return true;
            }
        return false;
    }

    private boolean isBlockedByTeamMainBot() {
        for (IRadarResult o : detectRadar())
            if (o.getObjectType() == IRadarResult.Types.TeamMainBot
                    && isInFront(o.getObjectDirection())
                    && !isBehind(o.getObjectDirection())
                    && o.getObjectDistance() < TEAMMATE_DETECTION_DISTANCE) {
                return true;
            }
        return false;
    }

    private boolean isBlockedByWreckObstacle() {
        for (IRadarResult o : detectRadar())
            if (o.getObjectType() == IRadarResult.Types.Wreck
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < WRECK_DETECTION_DISTANCE) {
                return true;
            }
        return false;
    }
}