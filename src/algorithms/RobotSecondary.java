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
        TURNING_BACK,          // used as generic "turn to targetAngle"
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

    // You can lower this later if you want tighter turns
    private static final double ANGLE_PRECISION = 0.03;

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

    private int yieldBackSteps = 0;
    private static final int YIELD_BACK_STEPS_MAIN = 6;
    private static final int YIELD_BACK_STEPS_SECONDARY = 3;
    private static final double YIELD_DISTANCE_SECONDARY = 140; // tweak 120–200

    // Roaming turning increment (30 degrees)
    private static final double ROAM_TURN_INCREMENT = Math.PI / 6; // 30°
    private int consecutiveBlocked = 0;
    private static final double ROAM_SCAN_DISTANCE = 250;
    private boolean lastMoveWasBack = false;
    private static final double ENEMY_MAINBOT_KEEP_DISTANCE = 400; // 550 is scan for Secondary, 350 for Main , so a bit more than main
    private static final int EVADE_BACK_STEPS = 8;
    private int evadeEnemySteps = 0;
    private double latchX = Double.NaN, latchY = Double.NaN;
    private boolean wallLatchActive = false;
    private boolean latchJustCompleted = false;
    private double WALLOFFSET= 400;


    private int enemyBroadcastCooldown = 0;
    private static final int ENEMY_BROADCAST_PERIOD = 50; // 50 steps
    private double mateX, mateY;
    private int teammateScanCooldown = 0;
    private static final int TEAMMATE_SCAN_PERIOD = 50; // 50 steps
    private static int DAMAGE_TAKEN_COOLDOWN = 100;
    private int damageTakenCooldown = 0;
    private double lastHealth = -1;

    // “go back to main bot” mode after taking damage
    private boolean retreatToMate = false;
    private int retreatCooldown = 0;
    private static final int RETREAT_COOLDOWN_STEPS = 80; // how long we try to retreat
    private static double retreatMateDistance = 200;
    private static double retreatDefaultX = 0;
    private static double retreatDefaultY = 0;
    private static double mateDistance = 0;

    // optional: don’t retreat if we’ve never seen the mate yet
    private boolean hasMatePos = false;

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

        retreatDefaultX = (Parameters.teamASecondaryBot1InitX + Parameters.teamASecondaryBot2InitX) / 2;
        retreatDefaultY = (Parameters.teamASecondaryBot1InitY + Parameters.teamASecondaryBot2InitY) / 2;
        if (botsAbove == 0 && botsBelow == 1) {
            // Bottom position
            role = Role.EXPLORER_ALPHA;
            robotName = "Explorer Alpha";
            System.out.println("I am " + robotName + " (bottom), I will explore the NORTH area.");
            state = State.TURNING_SOUTH;
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
        lastHealth = getHealth();
        consecutiveBlocked = 0;
        damageTakenCooldown = DAMAGE_TAKEN_COOLDOWN;
    }

    @Override
    public void step() {
        updateOdometry();
        readTeammateMessages();
        enemyBroadcastCooldown = Math.max(0, enemyBroadcastCooldown - 1);
        teammateScanCooldown = Math.max(0, teammateScanCooldown - 1);
        damageTakenCooldown = Math.max(0 , damageTakenCooldown - 1);

        if (teammateScanCooldown == 0) {
            scanTeamMates();
            teammateScanCooldown = TEAMMATE_SCAN_PERIOD;
        }

        damageTakenCheck();

// If retreating, do it with priority
        if (retreatToMate) {
            retreatCooldown--;
            if(mateDistance > retreatMateDistance){
                meetAtPoint(mateX, mateY, 100); // precision in mm (120 is reasonable)
            }else{
                meetAtPoint(retreatDefaultX, retreatDefaultY, 100);
            }
            if (retreatCooldown <= 0) retreatToMate = false;
            return;
        }


        switch (state){
            case TURNING_NORTH:
                if (isSameDirection(myGetHeading(), Parameters.NORTH)) {
                    state = State.GOING_NORTH;
                } else {
                    stepTurn(getTurnDirection(myGetHeading(), Parameters.NORTH));
                }
                break;

            case GOING_NORTH:
                if (detectWall()) {
                    if (latchJustCompleted) {
                        // We finished approaching this tick: turn now, don't re-latch
                        latchJustCompleted = false;
                        state = State.TURNING_WEST;
                        targetAngle = Parameters.WEST;
                        break;
                    }

                    if (!wallLatchActive) {
                        // First time we see the wall → latch + broadcast once
                        latchWallStart();
                        northBound = myY;
                        broadcastBorders("NORTH");
                    }

                    // Move toward the wall while latching
                    myMove();
                } else {
                    myMove();
                }
                break;

            case TURNING_SOUTH:
                if (isSameDirection(myGetHeading(), Parameters.SOUTH)) {
                    state = State.CHECKING_SOUTH;
                } else {
                    stepTurn(getTurnDirection(myGetHeading(), Parameters.SOUTH));
                }
                break;

            case CHECKING_SOUTH:
                if (detectWall()) {
                    if (latchJustCompleted) {
                        // We finished approaching this tick: turn now, don't re-latch
                        latchJustCompleted = false;
                        state = State.TURNING_EAST;
                        targetAngle = Parameters.EAST;
                        break;
                    }

                    if (!wallLatchActive) {
                        // First time we see the wall → latch + broadcast once
                        latchWallStart();
                        northBound = myY;
                        broadcastBorders("SOUTH");
                    }

                    // Move toward the wall while latching
                    myMove();
                } else {
                    myMove();
                }
                break;
            case TURNING_WEST:
                if (isSameDirection(myGetHeading(), Parameters.WEST)) {
                    state = State.CHECKING_WEST;
                } else {
                    stepTurn(getTurnDirection(myGetHeading(), Parameters.WEST));
                }
                break;

            case CHECKING_WEST:
                if (detectWall()) {
                    westBound = myX;
                    broadcastBorders("WEST");
                    /* Roamning normally after */
                    state = State.EXPLORATION_COMPLETE;
                } else {
                    myMove();
                }
                break;

            case TURNING_EAST:
                if (isSameDirection(myGetHeading(), Parameters.EAST)) {
                    state = State.CHECKING_EAST;
                } else {
                    stepTurn(getTurnDirection(myGetHeading(), Parameters.EAST));
                }
                break;

            case CHECKING_EAST:
                if (detectWall()) {
                    eastBound = myX;
                    broadcastBorders("EAST");
                    state = State.EXPLORATION_COMPLETE;
                } else {
                    myMove();
                }
                break;

            case EXPLORATION_COMPLETE:
                // Start roaming
                consecutiveBlocked = 0;
                state = State.MOVE;
                break;

            case MOVE:
                // 0) Enemy panic-back already in progress (HIGHEST PRIORITY)
                if (evadeEnemySteps > 0) {
                    myMoveBack();
                    evadeEnemySteps--;
                    break;
                }

                // Detect any enemy via radar
                IRadarResult enemy = getFirstEnemy();

                // A) If enemy visible -> broadcast (cooldown protected)
                if (enemy != null && enemyBroadcastCooldown == 0) {
                    broadcastEnemyPosition(enemy);
                    enemyBroadcastCooldown = ENEMY_BROADCAST_PERIOD;
                }

                // B) If enemy too close -> run away
                if (enemy != null && enemy.getObjectDistance() < ENEMY_MAINBOT_KEEP_DISTANCE) {
                    evadeEnemySteps = EVADE_BACK_STEPS;
                    targetAngle = normalize(enemy.getObjectDirection() + Math.PI);
                    state = State.TURNING_BACK;
                    break;
                }

                // 1) Yielding in progress (for allies)
                if (yieldBackSteps > 0) {
                    myMoveBack();
                    yieldBackSteps--;
                    break;
                }

                // 2) Our main bot has priority
                if (isBlockedByTeamMainBotOnlyFront()) {
                    yieldBackSteps = YIELD_BACK_STEPS_MAIN;
                    break;
                }

                // 3) Secondary-vs-secondary deadlock breaker
                if (isSecondaryBlockingFront()) {
                    if (role == Role.EXPLORER_ALPHA) {
                        yieldBackSteps = YIELD_BACK_STEPS_SECONDARY;
                    } else {
                        targetAngle = normalize(myGetHeading() + ROAM_TURN_INCREMENT);
                        state = State.TURNING_BACK;
                    }
                    break;
                }

                // 4) Normal roaming
                if (!blockedAhead()) {
                    myMove();
                    consecutiveBlocked = 0;
                } else {
                    consecutiveBlocked++;

                    targetAngle = normalize(myGetHeading() + ROAM_TURN_INCREMENT);

                    if (consecutiveBlocked % 8 == 0) {
                        targetAngle = normalize(myGetHeading() - ROAM_TURN_INCREMENT);
                    }
                    if (consecutiveBlocked > 30) {
                        targetAngle = normalize(myGetHeading() + Math.PI);
                        consecutiveBlocked = 0;
                    }

                    state = State.TURNING_BACK;
                }
                break;

            case TURNING_BACK:
                // Turn until we reach the 30° target
                if (!isSameDirection(myGetHeading(), targetAngle)) {
                    stepTurn(getTurnDirection(myGetHeading(), targetAngle));
                } else {
                    // After finishing this 30° chunk, immediately try moving again
                    state = State.MOVE;
                }
                break;


            case IDLE:
                // do nothing
                break;
        }
    }

    private Parameters.Direction getTurnDirection(double current, double target) {
        double diff = normalize(target - current);
        return (diff <= Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
    }

    private void myMove() {
        isMoving = true;
        lastMoveWasBack = false;
        move();
    }

    private void myMoveBack() {
        isMoving = true;
        lastMoveWasBack = true;
        moveBack();
    }


    private void updateOdometry() {
        boolean blockedByWall = detectWall();
        boolean blockedByWreck = isBlockedByWreckObstacle();
        boolean blockedByTeamMate = isBlockedByTeamMate();
        boolean blockedByOpponent = isBlockedByOpponent();

        if (isMoving) {
            if (!blockedByWall && !blockedByWreck && !blockedByTeamMate && !blockedByOpponent) {
                double s = Parameters.teamASecondaryBotSpeed;
                if (lastMoveWasBack) s = -s;
                myX += s * Math.cos(myGetHeading());
                myY += s * Math.sin(myGetHeading());
            }
            isMoving = false;
        }

    }

    private boolean isSameDirection(double dir1, double dir2) {
        return Math.abs(normalize(dir1) - normalize(dir2)) < ANGLE_PRECISION;
    }
    private void damageTakenCheck() {
        double h = getHealth();
        if (lastHealth < 0) lastHealth = h;

        // only trigger on DAMAGE (health goes down)
        if (h < lastHealth && damageTakenCooldown == 0) {
            if (hasMatePos) {
                retreatToMate = true;
                retreatCooldown = RETREAT_COOLDOWN_STEPS;
                damageTakenCooldown = DAMAGE_TAKEN_COOLDOWN;
            }
        }

        lastHealth = h;
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
    BROADCAST FUNCTIONS
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
    private void broadcastEnemyPosition(IRadarResult enemy){
        double enemyAbsoluteX = myX + enemy.getObjectDistance() * Math.cos(enemy.getObjectDirection());
        double enemyAbsoluteY = myY + enemy.getObjectDistance() * Math.sin(enemy.getObjectDirection());

        // Broadcast both spotter position AND enemy position for smart convergence
        String message = "SCOUT_ENEMY_LOCATION|" + robotName + "|" +
                (int)myX + "|" + (int)myY + "|" +
                (int)enemyAbsoluteX + "|" + (int)enemyAbsoluteY;
        broadcast(message);
        sendLogMessage(robotName + " broadcasting: I'm at (" + (int)myX + "," + (int)myY +
                "), enemy at (" + (int)enemyAbsoluteX + "," + (int)enemyAbsoluteY + ")");

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
//                        sendLogMessage(robotName + " received border info: " + borderType + " at " + pos);
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
                    // ignore invalid message
                }
            }
        }
    }
    private void meetAtPoint(double x, double y, double precision) {
        double distance = Math.hypot(x - myX, y - myY);

        if (distance < precision) {
            retreatToMate = false; // arrived
            state = State.MOVE;
            return;
        }

        double angleToTarget = Math.atan2(y - myY, x - myX);

        if (!isSameDirection(myGetHeading(), angleToTarget)) {
            Parameters.Direction dir = getTurnDirection(myGetHeading(), angleToTarget);
            stepTurn(dir);
            return;
        }

        // Move if possible; if blocked, just do your roam-turn behavior
        if (!blockedAhead()) {
            myMove();
        } else {
            targetAngle = normalize(myGetHeading() + ROAM_TURN_INCREMENT);
            state = State.TURNING_BACK;
        }
    }

    private void scanTeamMates() {
        double bestD = Double.POSITIVE_INFINITY;
        double bestX = mateX, bestY = mateY;
        boolean found = false;

        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.TeamMainBot) {
                double d = o.getObjectDistance();
                if (d < bestD) {
                    bestD = d;
                    found = true;
                    mateDistance = o.getObjectDistance();
                    // This simplifies to using absolute direction:
                    double dir = o.getObjectDirection();
                    bestX = myX + d * Math.cos(dir);
                    bestY = myY + d * Math.sin(dir);
                }
            }
        }

        if (found) {
            mateX = bestX;
            mateY = bestY;
            hasMatePos = true;
        }
    }



    /*
    DETECTION FUNCTIONS
     */
    private boolean detectWall() {
        if (wallLatchActive) {
            double traveled = dist(myX, myY, latchX, latchY);
            if (traveled >= WALLOFFSET) {
                wallLatchActive = false;
                latchJustCompleted = true;   // <- mark completion
                return true;
            }
            return false;
        }
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

    private boolean blockedAhead() {
        return detectWall()
                || isBlockedByWreckObstacle()
                || isBlockedByTeamMate()
                || isBlockedByOpponent();
    }
    private boolean isBlockedByTeamMainBotOnlyFront() {
        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.TeamMainBot
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < 200) {   // tweak distance if needed
                return true;
            }
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
    private boolean isSecondaryBlockingFront() {
        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.TeamSecondaryBot
                    || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot)
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < YIELD_DISTANCE_SECONDARY) {
                return true;
            }
        }
        return false;
    }

    private IRadarResult getClosestEnemyMainBot() {
        IRadarResult best = null;
        double bestD = Double.POSITIVE_INFINITY;

        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.OpponentMainBot) {
                double d = o.getObjectDistance();
                if (d < bestD) {
                    bestD = d;
                    best = o;
                }
            }
        }
        return best;
    }

    private boolean enemyMainBotTooClose() {
        IRadarResult e = getClosestEnemyMainBot();
        return e != null && e.getObjectDistance() < ENEMY_MAINBOT_KEEP_DISTANCE;
    }
    private double computeEvadeAngleFromEnemy(IRadarResult enemy) {
        // We want to face away from the enemy.
        // If enemy direction is absolute, desired heading is enemyDir + PI.
        // If it's relative, this still works reasonably since you normalize with heading elsewhere.
        return normalize(enemy.getObjectDirection() + Math.PI);
    }


    /*
    UTILITY FUNCTIONS

     */
    private double dist(double x1, double y1, double x2, double y2) {
        double dx = x1 - x2, dy = y1 - y2;
        return Math.sqrt(dx*dx + dy*dy);
    }

    private void latchWallStart() {
        latchX = myX;
        latchY = myY;
        wallLatchActive = true;
    }

    private IRadarResult getFirstEnemy() {
        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.OpponentMainBot
                    || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                return o;
            }
        }
        return null;
    }


}
