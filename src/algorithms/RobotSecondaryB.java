package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;

public class RobotSecondaryB extends Brain {

    private enum Role { UNDEFINED, EXPLORER_ALPHA, EXPLORER_BETA }
    private enum State {
        MOVE,
        TURNING_NORTH, TURNING_SOUTH, TURNING_WEST, TURNING_EAST,
        TURNING_BACK,          // generic "turn to targetAngle"
        GOING_NORTH, CHECKING_SOUTH, CHECKING_WEST, CHECKING_EAST,
        EXPLORATION_COMPLETE,
        UTURN,
        IDLE
    }

    private Role role = Role.UNDEFINED;
    private State state = State.IDLE;
    private State afterTurnState = State.MOVE;

    private String robotName = "undefined";
    private double myX, myY;
    private boolean isMoving = false;
    private boolean lastMoveWasBack = false;

    private static final double ANGLE_PRECISION = 0.03;
    private double targetAngle;

    // discovered bounds
    private double northBound = -1, southBound = -1, westBound = -1, eastBound = -1;

    private static final double DETECTION_RANGE = Parameters.teamBSecondaryBotFrontalDetectionRange;

    // ===== IMPROVED "SIMPLE" AVOIDANCE (incremental turns) =====
    private static final double AVOID_STEP = Math.PI / 6;      // 30°
    private static final int AVOID_MAX_STEPS = 12;             // 12 * 30° = 360°
    private int avoidSide = 1;                                 // +1 right, -1 left
    private int consecutiveBlocks = 0;

    private static final int BLOCK_ESCAPE_TRIGGER = 5;
    private int escapeBackSteps = 0;
    private static final int ESCAPE_BACK_STEPS = 3;

    private static final double ESCAPE_TURN = Math.PI / 2; // 90°

    private static final double WALL_BORDER_MARGIN = 120;
    private static final double SECONDARY_CLOSER_MARGIN = 300;

    // ===== WALL U-TURN SCAN (FIXED) =====
    private int uTurnStep = 0;          // 0..AVOID_MAX_STEPS
    private double uTurnStartHeading = 0;
    private int uTurnSide = 1;          // +1 right, -1 left

    // ===== teammate-secondary deadlock fixes =====
    private static final double FRONT_CONE = Math.PI / 3; // 60°
    private static final double TEAM_SECONDARY_RANGE = DETECTION_RANGE;
    private boolean blockedByTeammateSecondary = false;

    // ===== ENEMY BROADCASTING =====
    private int enemyBroadcastCd = 0;
    private double lastEnemyBX = -1, lastEnemyBY = -1;
    private static final int ENEMY_BROADCAST_CD_STEPS = 25;
    private static final double ENEMY_BROADCAST_MIN_MOVE = 120;
    private static final double ENEMY_BROADCAST_MAX_DIST = 700;


    @Override
    public void activate() {
        boolean seesNorth = false;
        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) seesNorth = true;
            }
        }

        if (seesNorth) {
            role = Role.EXPLORER_ALPHA;
            robotName = "Explorer Alpha";
            myX = Parameters.teamBSecondaryBot2InitX;
            myY = Parameters.teamBSecondaryBot2InitY;
            state = State.TURNING_SOUTH;
            targetAngle = Parameters.SOUTH;
        } else {
            role = Role.EXPLORER_BETA;
            robotName = "Explorer Beta";
            myX = Parameters.teamBSecondaryBot1InitX;
            myY = Parameters.teamBSecondaryBot1InitY;
            state = State.TURNING_NORTH;
            targetAngle = Parameters.NORTH;
        }

        // UTURN init
        uTurnStep = 0;
        uTurnStartHeading = 0;
        uTurnSide = 1;

        isMoving = false;
        avoidSide = 1;
        consecutiveBlocks = 0;
        escapeBackSteps = 0;
        blockedByTeammateSecondary = false;
    }

    @Override
    public void step() {
        updateOdometry();
        readTeammateMessages();
        if (enemyBroadcastCd > 0) enemyBroadcastCd--;

        IRadarResult bestEnemy = null;
        double bestD = Double.POSITIVE_INFINITY;

        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() != IRadarResult.Types.OpponentMainBot &&
                    o.getObjectType() != IRadarResult.Types.OpponentSecondaryBot) continue;

            if (o.getObjectDistance() < bestD) {
                bestD = o.getObjectDistance();
                bestEnemy = o;
            }
        }

        if (bestEnemy != null && bestD < ENEMY_BROADCAST_MAX_DIST && enemyBroadcastCd == 0) {
            double ex = myX + bestEnemy.getObjectDistance() * Math.cos(bestEnemy.getObjectDirection());
            double ey = myY + bestEnemy.getObjectDistance() * Math.sin(bestEnemy.getObjectDirection());

            if (lastEnemyBX < 0 || Math.hypot(ex - lastEnemyBX, ey - lastEnemyBY) > ENEMY_BROADCAST_MIN_MOVE) {
                broadcastEnemyPosition(bestEnemy);
                lastEnemyBX = ex;
                lastEnemyBY = ey;
                enemyBroadcastCd = ENEMY_BROADCAST_CD_STEPS;
            }
        }

        if (escapeBackSteps > 0) {
            myMoveBack();
            escapeBackSteps--;
            return;
        }

        switch (state) {

            case TURNING_NORTH:
                turnToward(Parameters.NORTH, State.GOING_NORTH);
                break;

            case TURNING_SOUTH:
                turnToward(Parameters.SOUTH, State.CHECKING_SOUTH);
                break;

            case TURNING_WEST:
                turnToward(Parameters.WEST, State.CHECKING_WEST);
                break;

            case TURNING_EAST:
                turnToward(Parameters.EAST, State.CHECKING_EAST);
                break;

            case UTURN:
                doUTurnScan();
                break;

            case GOING_NORTH:
                if (detectWall()) {
                    northBound = myY - DETECTION_RANGE;
                    broadcastBorders("NORTH");
                    state = State.TURNING_WEST;
                    targetAngle = Parameters.WEST;
                    break;
                }
                if (blockedAhead()) {
                    simpleAvoid(State.GOING_NORTH);
                    break;
                }
                myMove();
                break;

            case CHECKING_SOUTH:
                if (detectWall()) {
                    southBound = myY + DETECTION_RANGE;
                    broadcastBorders("SOUTH");
                    state = State.TURNING_EAST;
                    targetAngle = Parameters.EAST;
                    break;
                }
                if (blockedAhead()) {
                    simpleAvoid(State.CHECKING_SOUTH);
                    break;
                }
                myMove();
                break;

            case CHECKING_WEST:
                if (detectWall()) {
                    westBound = myX - DETECTION_RANGE;
                    broadcastBorders("WEST");
                    state = State.EXPLORATION_COMPLETE;
                    break;
                }
                if (blockedAhead()) {
                    simpleAvoid(State.CHECKING_WEST);
                    break;
                }
                myMove();
                break;

            case CHECKING_EAST:
                if (detectWall()) {
                    eastBound = myX + DETECTION_RANGE;
                    broadcastBorders("EAST");
                    state = State.EXPLORATION_COMPLETE;
                    break;
                }
                if (blockedAhead()) {
                    simpleAvoid(State.CHECKING_EAST);
                    break;
                }
                myMove();
                break;

            case EXPLORATION_COMPLETE:
                consecutiveBlocks = 0;
                state = State.MOVE;
                break;

            case MOVE:
                simpleAvoid(State.MOVE);
                break;

            case TURNING_BACK:
                if (!isSameDirection(myGetHeading(), targetAngle)) {
                    stepTurn(getTurnDirection(myGetHeading(), targetAngle));
                } else {
                    state = afterTurnState;
                }
                break;

            case IDLE:
                break;
        }
    }

    // =========================
    // SIMPLE AVOIDANCE (UPGRADED)
    // =========================
    private void simpleAvoid(State returnState) {

        // anti-spam guard: only UTURN if we really failed at least once
        if (detectWall() && consecutiveBlocks >= 1) {
            enterUTurnMode(returnState);
            return;
        }

        if (!blockedAhead()) {
            myMove();
            consecutiveBlocks = 0;
            return;
        }

        consecutiveBlocks++;

        // If it's OUR other secondary bot, don't mirror each other:
        // Alpha always yields one side, Beta the other.
        if (blockedByTeammateSecondary) {
            avoidSide = (role == Role.EXPLORER_ALPHA) ? +1 : -1;
        }

        if (consecutiveBlocks >= BLOCK_ESCAPE_TRIGGER) {
            escapeBackSteps = ESCAPE_BACK_STEPS;

            avoidSide = -avoidSide;
            targetAngle = normalize(myGetHeading() + avoidSide * ESCAPE_TURN);

            afterTurnState = returnState;
            state = State.TURNING_BACK;

            consecutiveBlocks = 0;
            return;
        }

        if (consecutiveBlocks > AVOID_MAX_STEPS) {
            myMoveBack();
            consecutiveBlocks = 0;
            avoidSide = -avoidSide;
            return;
        }

        // Only flip side for symmetric deadlocks if NOT teammate-secondary
        if (!blockedByTeammateSecondary && consecutiveBlocks % 2 == 0) avoidSide = -avoidSide;

        double turnAmount = consecutiveBlocks * AVOID_STEP;
        targetAngle = normalize(myGetHeading() + avoidSide * turnAmount);

        afterTurnState = returnState;
        state = State.TURNING_BACK;
    }

    // ==========================================================
    // UTURN MODE (FIXED)
    // ==========================================================
    private void enterUTurnMode(State returnState) {
        escapeBackSteps = Math.max(escapeBackSteps, 1);

        uTurnSide = -uTurnSide;

        uTurnStartHeading = normalize(myGetHeading() + Math.PI);
        uTurnStep = 0;
        targetAngle = uTurnStartHeading;

        afterTurnState = returnState;
        state = State.UTURN;
    }

    private void doUTurnScan() {
        if (!isSameDirection(myGetHeading(), targetAngle)) {
            stepTurn(getTurnDirection(myGetHeading(), targetAngle));
            return;
        }

        // aligned: if front is free, move and exit
        if (detectFront().getObjectType() == IFrontSensorResult.Types.NOTHING) {
            myMove();
            consecutiveBlocks = 0;
            state = afterTurnState;
            return;
        }

        // next step
        uTurnStep++;

        if (uTurnStep > AVOID_MAX_STEPS) {
            escapeBackSteps = Math.max(escapeBackSteps, 2);
            consecutiveBlocks = 0;
            state = afterTurnState;
            return;
        }

        targetAngle = normalize(uTurnStartHeading + uTurnSide * uTurnStep * AVOID_STEP);
    }

    // =========================
    // MOVEMENT + ODOMETRY
    // =========================
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
        if (!isMoving) return;

        boolean blocked = blockedAhead();
        if (!blocked) {
            double s = Parameters.teamBSecondaryBotSpeed;
            if (lastMoveWasBack) s = -s;

            myX += s * Math.cos(myGetHeading());
            myY += s * Math.sin(myGetHeading());
            sendLogMessage(robotName + " (x=" + (int) myX + ", y=" + (int) myY + ")");
        }

        isMoving = false;
    }

    // =========================
    // TURN HELPERS
    // =========================
    private void turnToward(double angle, State next) {
        if (isSameDirection(myGetHeading(), angle)) {
            state = next;
        } else {
            stepTurn(getTurnDirection(myGetHeading(), angle));
        }
    }

    private Parameters.Direction getTurnDirection(double current, double target) {
        double diff = normalize(target - current);
        return (diff <= Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
    }

    private boolean isSameDirection(double dir1, double dir2) {
        return Math.abs(normalize(dir1) - normalize(dir2)) < ANGLE_PRECISION;
    }

    private double myGetHeading() {
        double h = getHeading();
        while (h < 0) h += 2 * Math.PI;
        while (h >= 2 * Math.PI) h -= 2 * Math.PI;
        return h;
    }

    private double normalize(double dir) {
        double res = dir;
        while (res < 0) res += 2 * Math.PI;
        while (res >= 2 * Math.PI) res -= 2 * Math.PI;
        return res;
    }

    // =========================
    // BROADCAST + READ
    // =========================
    private void broadcastBorders(String border) {
        int val;
        switch (border) {
            case "NORTH": val = (int) northBound; break;
            case "SOUTH": val = (int) southBound; break;
            case "WEST":  val = (int) westBound;  break;
            case "EAST":  val = (int) eastBound;  break;
            default: return;
        }
        broadcast("BORDER|" + border + "|" + val);
    }

    private void readTeammateMessages() {
        ArrayList<String> messages = fetchAllMessages();
        for (String msg : messages) {
            if (!msg.startsWith("BORDER")) continue;
            try {
                String[] parts = msg.split("\\|");
                if (parts.length != 3) continue;
                String borderType = parts[1];
                int pos = Integer.parseInt(parts[2]);

                switch (borderType) {
                    case "NORTH": northBound = pos; break;
                    case "SOUTH": southBound = pos; break;
                    case "WEST":  westBound  = pos; break;
                    case "EAST":  eastBound  = pos; break;
                }
            } catch (Exception ignored) {}
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


    // =========================
    // OBSTACLE DETECTION
    // =========================
    private boolean detectWall() {
        return detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
    }

    private boolean blockedAhead() {
        blockedByTeammateSecondary = false;

        if (wallBlocksNow()) return true;

        boolean wreck = isRadarObstacleInFront(IRadarResult.Types.Wreck, DETECTION_RANGE);
        boolean teamMain = isRadarObstacleInFront(IRadarResult.Types.TeamMainBot, DETECTION_RANGE);
        boolean oppMain = isRadarObstacleInFront(IRadarResult.Types.OpponentMainBot, DETECTION_RANGE);

        boolean oppSec = isRadarObstacleInFront(IRadarResult.Types.OpponentSecondaryBot, DETECTION_RANGE);

        boolean teamSec = isRadarObstacleInFront(IRadarResult.Types.TeamSecondaryBot, TEAM_SECONDARY_RANGE);
        if (teamSec) blockedByTeammateSecondary = true;

        return wreck || teamMain || oppMain || oppSec || teamSec;
    }

    private boolean isRadarObstacleInFront(IRadarResult.Types type, double range) {
        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() != type) continue;
            if (o.getObjectDistance() >= range) continue;
            if (isInFront(o.getObjectDirection())) return true;
        }
        return false;
    }

    private boolean isInFront(double objDir) {
        double rel = normalize(objDir - myGetHeading());
        return rel < FRONT_CONE || rel > (2 * Math.PI - FRONT_CONE);
    }

    private boolean wallBlocksNow() {
        if (!detectWall()) return false;

        if (northBound < 0 && southBound < 0 && westBound < 0 && eastBound < 0) return true;

        double h = myGetHeading();

        if (isSameDirection(h, Parameters.NORTH) && northBound >= 0) {
            return myY <= northBound + WALL_BORDER_MARGIN;
        }
        if (isSameDirection(h, Parameters.SOUTH) && southBound >= 0) {
            return myY >= southBound - WALL_BORDER_MARGIN;
        }
        if (isSameDirection(h, Parameters.WEST) && westBound >= 0) {
            return myX <= westBound + WALL_BORDER_MARGIN;
        }
        if (isSameDirection(h, Parameters.EAST) && eastBound >= 0) {
            return myX >= eastBound - WALL_BORDER_MARGIN;
        }

        return true;
    }
}
