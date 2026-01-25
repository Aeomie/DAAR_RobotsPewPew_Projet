package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

public class SimpleRobot extends Brain {

    private enum Role { WARIO, MARIO, LUIGI, UNDEFINED }
    private enum State { MOVE, TURNING, BACKING_UP, IDLE, UTURN }

    private State state = State.MOVE;

    // --- turning precision (for "am I aligned yet?") ---
    private static final double ANGLE_PRECISION = 0.05;

    // --- radar direction tolerance (for "is obstacle on this ray?") ---
    private static final double RADAR_DIR_EPS = 0.15; // ~8.6°

    private double targetAngle = 0;

    // --- avoidance steps ---
    private static final double AVOID_STEP = Math.PI / 6; // 30°
    private int avoidSide = 1; // +1 right, -1 left

    private int consecutiveBlocks = 0;
    private static final int BLOCK_ESCAPE_TRIGGER = 5;

    private int escapeBackSteps = 0;
    private static final int ESCAPE_BACK_STEPS = 3;

    // --- wall U-turn scan ---
    private int uTurnChecksLeft = 0;
    private double uTurnStartHeading = 0;
    private int uTurnSide = 1; // +1 right, -1 left

    // --- radar as REAL radius bubble (360°) ---
    private static final double RADAR_RADIUS = 150;

    // turning reason flags
    private boolean turnUsesRadarRayCheck = false;

    // after we found a free heading, commit 1 forward step
    private int commitForwardSteps = 0;

    // --- tiny odometry (optional) ---
    private double myX, myY;
    private boolean isMoving = false;
    private boolean lastMoveWasBack = false;

    private String robotName = "undefined";
    private Role role = Role.UNDEFINED;

    @Override
    public void activate() {
        identifyRole();

        state = State.MOVE;
        consecutiveBlocks = 0;
        escapeBackSteps = 0;
        avoidSide = 1;

        uTurnChecksLeft = 0;
        uTurnStartHeading = 0;
        uTurnSide = 1;

        commitForwardSteps = 0;
        turnUsesRadarRayCheck = false;
    }

    public void identifyRole() {
        boolean seesNorth = false, seesSouth = false;

        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.TeamMainBot) {
                if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) seesNorth = true;
                if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH)) seesSouth = true;
            }
        }

        int whoAmI = seesNorth && !seesSouth ? 3 : (seesSouth && !seesNorth ? 1 : 2);

        switch (whoAmI) {
            case 1:
                myX = Parameters.teamBMainBot1InitX;
                myY = Parameters.teamBMainBot1InitY;
                robotName = "WARIO";
                role = Role.WARIO;
                break;
            case 2:
                myX = Parameters.teamBMainBot2InitX;
                myY = Parameters.teamBMainBot2InitY;
                robotName = "MARIO";
                role = Role.MARIO;
                break;
            case 3:
                myX = Parameters.teamBMainBot3InitX;
                myY = Parameters.teamBMainBot3InitY;
                robotName = "LUIGI";
                role = Role.LUIGI;
                break;
            default:
                robotName = "UNKNOWN";
                role = Role.UNDEFINED;
        }
    }

    @Override
    public void step() {
        updateOdometry();

        if (escapeBackSteps > 0) {
            myMoveBack();
            escapeBackSteps--;
            return;
        }

        switch (state) {
            case MOVE:
                moveUsingFrontThenRadarRadius();
                break;

            case TURNING:
                doTurningWithRadarRayCheck();
                break;

            case UTURN:
                doUTurnScan();
                break;

            case BACKING_UP:
                myMoveBack();
                state = State.MOVE;
                break;

            case IDLE:
                break;
        }
    }

    // ==========================================================
    // MAIN MOVE RULE:
    // - If front is blocked => avoid (progressive) using this logic
    // - If front is safe => check RADAR_RADIUS (360°). If too close to anything, rotate bit-by-bit
    //   until the *current candidate direction* is not blocked by an obstacle ray, then move.
    // ==========================================================
    private void moveUsingFrontThenRadarRadius() {
        IFrontSensorResult front = detectFront();

        // If we just found a free heading previously, force 1 forward step (as long as front is clear)
        if (commitForwardSteps > 0 && front.getObjectType() == IFrontSensorResult.Types.NOTHING) {
            myMove();
            commitForwardSteps--;
            return;
        }

        // WALL => UTURN scan (360° search for a free direction)
        if (front.getObjectType() == IFrontSensorResult.Types.WALL) {
            enterUTurnMode();
            return;
        }

        // FRONT blocked (enemy bot / wreck / etc.)
        if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
            handleFrontBlockedNonWall();
            return;
        }

        // FRONT is clear => now RADAR RADIUS bubble (360°)
        if (radarHasAnythingWithinRadius(RADAR_RADIUS)) {
            // pick a side based on the closest obstacle around us (real radius), then start scanning
            Integer side = chooseSideFromClosestRadarWithinRadius(RADAR_RADIUS);
            if (side != null) avoidSide = side;

            // start with a small turn; TURNING will keep stepping until the chosen ray is no longer blocked
            targetAngle = normalize(myGetHeading() + avoidSide * AVOID_STEP);
            turnUsesRadarRayCheck = true;
            state = State.TURNING;
            return;
        }

        // Totally clear => move
        myMove();
        consecutiveBlocks = 0;
    }

    // FRONT blocked by something that isn't a wall
    private void handleFrontBlockedNonWall() {
        consecutiveBlocks++;

        // try to pick a side away from the closest radar obstacle (within radius), else alternate
        Integer side = chooseSideFromClosestRadarWithinRadius(RADAR_RADIUS);
        if (side != null) avoidSide = side;
        else if (consecutiveBlocks % 2 == 0) avoidSide = -avoidSide;

        // progressive target: 30, 60, 90, 120...
        double turnAmount = progressiveTurn(consecutiveBlocks);
        targetAngle = normalize(myGetHeading() + avoidSide * turnAmount);

        // safety escape
        if (consecutiveBlocks >= BLOCK_ESCAPE_TRIGGER) {
            escapeBackSteps = ESCAPE_BACK_STEPS;
            avoidSide = -avoidSide;
            // after backing, force a 90° change (3 * 30°)
            targetAngle = normalize(myGetHeading() + avoidSide * (3 * AVOID_STEP));
            consecutiveBlocks = 0;
        }

        turnUsesRadarRayCheck = true;
        state = State.TURNING;
    }

    // TURNING behavior:
    // - turn toward targetAngle
    // - when aligned, ask: "is this direction still blocked by a radar obstacle ray?"
    //   if yes -> rotate another 30° and try again
    //   if no  -> commit 1 forward step
    private void doTurningWithRadarRayCheck() {
        if (!isSameDirection(myGetHeading(), targetAngle)) {
            stepTurn(getTurnDirection(myGetHeading(), targetAngle));
            return;
        }

        // aligned
        if (turnUsesRadarRayCheck) {
            if (radarBlocksHeading(targetAngle, RADAR_RADIUS)) {
                // still blocked in this direction -> keep scanning
                targetAngle = normalize(targetAngle + avoidSide * AVOID_STEP);
                return;
            } else {
                // found a free ray -> move forward (next tick)
                commitForwardSteps = 1;
                turnUsesRadarRayCheck = false;
                state = State.MOVE;
                return;
            }
        }

        // no special check, just finish
        state = State.MOVE;
    }

    // ==========================================================
    // UTURN MODE (wall): start facing 180° away, then scan 360° in 30° steps.
    // if front becomes clear => move + exit
    // ==========================================================
    private void enterUTurnMode() {
        escapeBackSteps = Math.max(escapeBackSteps, 1);

        // alternate scan direction to avoid corner loops
        uTurnSide = -uTurnSide;

        // base direction: directly away from the wall (front saw wall)
        uTurnStartHeading = normalize(myGetHeading() + Math.PI);

        uTurnChecksLeft = (int) Math.ceil((2 * Math.PI) / AVOID_STEP);
        targetAngle = uTurnStartHeading;

        consecutiveBlocks = 0;
        turnUsesRadarRayCheck = false;
        state = State.UTURN;
    }

    private void doUTurnScan() {
        if (!isSameDirection(myGetHeading(), targetAngle)) {
            stepTurn(getTurnDirection(myGetHeading(), targetAngle));
            return;
        }

        if (detectFront().getObjectType() == IFrontSensorResult.Types.NOTHING) {
            // found a free direction
            commitForwardSteps = 1;
            state = State.MOVE;
            return;
        }

        uTurnChecksLeft--;
        if (uTurnChecksLeft <= 0) {
            // full scan failed: back up more and try moving logic again
            escapeBackSteps = Math.max(escapeBackSteps, 2);
            state = State.MOVE;
            return;
        }

        int totalSteps = (int) Math.ceil((2 * Math.PI) / AVOID_STEP);
        int stepIndex = totalSteps - uTurnChecksLeft + 1;
        targetAngle = normalize(uTurnStartHeading + uTurnSide * stepIndex * AVOID_STEP);
    }

    // ==========================================================
    // RADAR HELPERS (REAL RADIUS + RAY-BLOCK CHECK)
    // ==========================================================
    private boolean radarHasAnythingWithinRadius(double radius) {
        for (IRadarResult o : detectRadar()) {
            if (!isWantedRadarType(o.getObjectType())) continue;
            if (o.getObjectDistance() < radius) return true;
        }
        return false;
    }

    private boolean radarBlocksHeading(double candidateHeading, double radius) {
        for (IRadarResult o : detectRadar()) {
            if (!isWantedRadarType(o.getObjectType())) continue;
            if (o.getObjectDistance() >= radius) continue;

            // obstacle lies on this candidate ray => blocked
            if (isSameDirectionRadar(o.getObjectDirection(), candidateHeading)) return true;
        }
        return false;
    }

    // pick side away from the closest obstacle inside radius (360°)
    // returns -1 (turn left) or +1 (turn right) or null if none
    private Integer chooseSideFromClosestRadarWithinRadius(double radius) {
        IRadarResult closest = null;
        double best = Double.POSITIVE_INFINITY;

        for (IRadarResult o : detectRadar()) {
            if (!isWantedRadarType(o.getObjectType())) continue;
            double d = o.getObjectDistance();
            if (d >= radius) continue;

            if (d < best) {
                best = d;
                closest = o;
            }
        }

        if (closest == null) return null;

        double rel = normalize(closest.getObjectDirection() - myGetHeading());
        // object on right (0..pi) => turn left (-1). object on left => turn right (+1)
        return (rel > 0 && rel < Math.PI) ? -1 : +1;
    }

    private boolean isWantedRadarType(IRadarResult.Types t) {
        return (t == IRadarResult.Types.Wreck)
                || (t == IRadarResult.Types.TeamMainBot)
                || (t == IRadarResult.Types.OpponentMainBot)
                || (t == IRadarResult.Types.TeamSecondaryBot)
                || (t == IRadarResult.Types.OpponentSecondaryBot);
    }

    private boolean isSameDirectionRadar(double dir1, double dir2) {
        return Math.abs(normalize(dir1) - normalize(dir2)) < RADAR_DIR_EPS;
    }

    // progressive turn: 30, 60, 90, 120...
    private double progressiveTurn(int blocks) {
        double turn = blocks * AVOID_STEP;
        double maxTurn = 2 * Math.PI - AVOID_STEP; // up to 330°
        if (turn > maxTurn) turn = maxTurn;
        return turn;
    }

    // ==========================================================
    // MOVEMENT + ODOMETRY
    // ==========================================================
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

        if (detectFront().getObjectType() != IFrontSensorResult.Types.WALL) {
            double s = Parameters.teamBMainBotSpeed;
            if (lastMoveWasBack) s = -s;

            myX += s * Math.cos(myGetHeading());
            myY += s * Math.sin(myGetHeading());
            sendLogMessage(robotName + " (x=" + (int) myX + ", y=" + (int) myY + ")");
        }

        isMoving = false;
    }

    // ==========================================================
    // TURN HELPERS
    // ==========================================================
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
}