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
    // 30°, 60°, 90°, 120°... when blocked repeatedly
    private static final double AVOID_STEP = Math.PI / 6;      // 30°
    private static final int AVOID_MAX_STEPS = 12;             // 12 * 30° = 360°
    private int avoidSide = 1;                                 // +1 right, -1 left
    private int consecutiveBlocks = 0;

    private static final int BLOCK_ESCAPE_TRIGGER = 5;
    private int escapeBackSteps = 0;
    private static final int ESCAPE_BACK_STEPS = 3;

    // big “commitment” turn when wedged
    private static final double ESCAPE_TURN = Math.PI / 2; // 90°

    private static final double WALL_BORDER_MARGIN = 120;
    private static final double SECONDARY_CLOSER_MARGIN = 300;

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

        isMoving = false;
        avoidSide = 1;
        consecutiveBlocks = 0;
        escapeBackSteps = 0;
    }

    @Override
    public void step() {
        updateOdometry();
        readTeammateMessages();

        // hard escape: if we triggered it, back up first
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
                // roaming uses the same avoidance rule
                simpleAvoid(State.MOVE);
                break;

            case TURNING_BACK:
                if (!isSameDirection(myGetHeading(), targetAngle)) {
                    stepTurn(getTurnDirection(myGetHeading(), targetAngle));
                } else {
                    // IMPORTANT: don't reset counters here.
                    // Next tick in afterTurnState, we'll either move (if clear) or call simpleAvoid again and increase turn.
                    state = afterTurnState;
                }
                break;

            case IDLE:
                break;
        }
    }

    // =========================
    // SIMPLE AVOIDANCE (UPGRADED)
    // - If blocked repeatedly: try 30°, 60°, 90°, 120°... around
    // - If full scan fails: backstep once + flip side + reset
    // - If "hard stuck": backstep burst + 90° commitment
    // =========================
    private void simpleAvoid(State returnState) {

        // Clear -> move and reset counters
        if (!blockedAhead()) {
            myMove();
            consecutiveBlocks = 0;
            return;
        }

        // Blocked this tick
        consecutiveBlocks++;

        // HARD ESCAPE if stuck too long
        if (consecutiveBlocks >= BLOCK_ESCAPE_TRIGGER) {
            escapeBackSteps = ESCAPE_BACK_STEPS;

            avoidSide = -avoidSide; // alternate
            targetAngle = normalize(myGetHeading() + avoidSide * ESCAPE_TURN);

            afterTurnState = returnState;
            state = State.TURNING_BACK;

            consecutiveBlocks = 0; // reset so we don't instantly re-trigger
            return;
        }

        // If we've effectively scanned a full circle in 30° chunks, do a soft reset
        if (consecutiveBlocks > AVOID_MAX_STEPS) {
            myMoveBack();               // one backstep to create space
            consecutiveBlocks = 0;
            avoidSide = -avoidSide;
            return;
        }

        // Progressive turn: 30°, 60°, 90°, 120°...
        // Flip side occasionally so we don't orbit forever in symmetric deadlocks
        if (consecutiveBlocks % 2 == 0) avoidSide = -avoidSide;

        double turnAmount = consecutiveBlocks * AVOID_STEP;
        targetAngle = normalize(myGetHeading() + avoidSide * turnAmount);

        afterTurnState = returnState;
        state = State.TURNING_BACK;
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

    // =========================
    // OBSTACLE DETECTION
    // =========================
    private boolean detectWall() {
        return detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
    }

    private boolean blockedAhead() {
        // wall is handled below (with border logic)
        if (wallBlocksNow()) return true;

        return isRadarObstacleInFront(IRadarResult.Types.Wreck, DETECTION_RANGE)
                || isRadarObstacleInFront(IRadarResult.Types.TeamMainBot, DETECTION_RANGE)
                || isRadarObstacleInFront(IRadarResult.Types.OpponentMainBot, DETECTION_RANGE)
                || isRadarObstacleInFront(IRadarResult.Types.OpponentSecondaryBot,
                Math.max(0, DETECTION_RANGE - SECONDARY_CLOSER_MARGIN))
                || isRadarObstacleInFront(IRadarResult.Types.TeamSecondaryBot,
                Math.max(0, DETECTION_RANGE - SECONDARY_CLOSER_MARGIN));
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
        return rel < Math.PI / 4 || rel > (2 * Math.PI - Math.PI / 4);
    }

    private boolean wallBlocksNow() {
        if (!detectWall()) return false;

        // If we don't know any border yet, wall blocks normally
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
