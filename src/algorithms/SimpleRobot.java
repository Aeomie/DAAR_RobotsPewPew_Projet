package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;

public class SimpleRobot extends Brain {

    private enum Role { WARIO, MARIO, LUIGI, UNDEFINED }
    private enum State { MOVE, TURNING, BACKING_UP,
        IDLE, UTURN, CONVERGING, TEST_STOPPED }

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
    private State afterTurnState = State.MOVE;

    // Map bounds discovered (kept for your existing comms)
    private double northBound = -1;
    private double westBound = -1;
    private double eastBound = -1;
    private double southBound = -1;

    // UPDATE ENEMY LOCATION COMMUNICATIONS
    private static final double ENEMY_UPDATE_COOLDOWN = 1000; // start high to avoid immediate use
    private int stepsSinceEnemyUpdate = 0;
    private double currentTargetX = -1;
    private double currentTargetY = -1;
    private static final double FLANK_OFFSET_X = 150;
    private static final double TARGET_PRECISION = 50;


    private int test_time = 300;
    @Override
    public void activate() {
        identifyRole();

        state = State.TEST_STOPPED;
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
        readTeammateMessages();

        test_time = Math.max(0, test_time - 1);
        stepsSinceEnemyUpdate++;
        if (stepsSinceEnemyUpdate > ENEMY_UPDATE_COOLDOWN) {
            currentTargetX = -1;
            currentTargetY = -1;
            if (state == State.CONVERGING) state = State.MOVE;
        }
        if (escapeBackSteps > 0) {
            myMoveBack();
            escapeBackSteps--;
            return;
        }

        switch (state) {
            case TEST_STOPPED:
                if (test_time <= 0) {
                    currentTargetY = 150;
                    currentTargetX = 1500;
                    state = State.CONVERGING;
                }
                break;
            case MOVE:
                if (currentTargetX != -1 && currentTargetY != -1) {
                    state = State.CONVERGING;
                    break;
                }
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
            case CONVERGING:
                if (commitForwardSteps > 0 && detectFront().getObjectType() == IFrontSensorResult.Types.NOTHING) {
                    myMove();
                    commitForwardSteps--;
                    break;
                }

                if (currentTargetX != -1 && currentTargetY != -1) {
                    meetAtPoint(currentTargetX, currentTargetY, TARGET_PRECISION);
                } else {
                    state = State.MOVE;
                }
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
            afterTurnState = State.MOVE;
            enterUTurnMode();
            return;
        }

        // FRONT blocked (enemy bot / wreck / etc.)
        if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
            afterTurnState = State.MOVE;
            handleFrontBlockedNonWall();
            return;
        }

        // FRONT is clear => now RADAR RADIUS bubble (360°)
        if (radarHasAnythingWithinRadius(RADAR_RADIUS)) {
            afterTurnState = State.MOVE;
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

    /* ==========================================================
        go to Point & reading messages
     */
    private void meetAtPoint(double x, double y, double precision) {
        if (x == -1 || y == -1) {
            state = State.MOVE;
            return;
        }

        double distance = Math.hypot(x - myX, y - myY);
        if (distance < precision) {
            sendLogMessage(robotName + " >>> Arrived near target!");
            state = State.MOVE;
            return;
        }

        double angleToTarget = normalize(Math.atan2(y - myY, x - myX));

        // turn toward target first
        if (!isSameDirection(myGetHeading(), angleToTarget)) {
            targetAngle = angleToTarget;
            turnUsesRadarRayCheck = false;     // pure turning, not scanning
            afterTurnState = State.CONVERGING; // come back here after turn
            state = State.TURNING;
            return;
        }

        // aligned: if wall -> UTURN (then come back to converging)
        IFrontSensorResult front = detectFront();
        if (front.getObjectType() == IFrontSensorResult.Types.WALL) {
            afterTurnState = State.CONVERGING;
            enterUTurnMode();
            return;
        }

        // aligned: if blocked by something -> progressive avoid (then come back to converging)
        if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
            afterTurnState = State.CONVERGING;
            handleFrontBlockedNonWall();
            return;
        }

        // optional: radar bubble avoidance while converging too
        if (radarHasAnythingWithinRadius(RADAR_RADIUS)) {
            Integer side = chooseSideFromClosestRadarWithinRadius(RADAR_RADIUS);
            if (side != null) avoidSide = side;

            targetAngle = normalize(myGetHeading() + avoidSide * AVOID_STEP);
            turnUsesRadarRayCheck = true;
            afterTurnState = State.CONVERGING;
            state = State.TURNING;
            return;
        }

        // clear -> move
        myMove();
        consecutiveBlocks = 0;
    }

    private void broadcastEnemyPosition(IRadarResult enemy) {
        double enemyAbsoluteX = myX + enemy.getObjectDistance() * Math.cos(enemy.getObjectDirection());
        double enemyAbsoluteY = myY + enemy.getObjectDistance() * Math.sin(enemy.getObjectDirection());

        String message = "ENEMY_LOCATION|" + robotName + "|" +
                (int) myX + "|" + (int) myY + "|" +
                (int) enemyAbsoluteX + "|" + (int) enemyAbsoluteY;
        broadcast(message);
    }

    private void readTeammateMessages() {
        ArrayList<String> messages = fetchAllMessages();

        for (String msg : messages) {
            if (msg.startsWith("ENEMY_LOCATION|")) {
                try {
                    String[] parts = msg.split("\\|");
                    if (parts.length == 6) {
                        String spotter = parts[1];
                        if (robotName.equals(spotter)) continue;

                        double enemyX = Double.parseDouble(parts[4]);
                        double enemyY = Double.parseDouble(parts[5]);
                        stepsSinceEnemyUpdate = 0;
                        sendLogMessage(robotName + " ENEMY from " + spotter+
                                " (x=" + (int) enemyX + ", y=" + (int) enemyY + ")");
                        applyFormationOffset(spotter, enemyX, enemyY);
                    }
                } catch (Exception ignored) {}
            }

            if (msg.startsWith("BORDER")) {
                try {
                    String[] parts = msg.split("\\|");
                    if (parts.length == 3) {
                        String borderType = parts[1];
                        int pos = Integer.parseInt(parts[2]);
                        switch (borderType) {
                            case "NORTH": northBound = pos; break;
                            case "SOUTH": southBound = pos; break;
                            case "WEST":  westBound = pos;  break;
                            case "EAST":  eastBound = pos;  break;
                        }
                    }
                } catch (Exception ignored) {}
            }

            if (msg.startsWith("SCOUT_ENEMY_LOCATION")) {
                try {
                    String[] parts = msg.split("\\|");
                    if (parts.length == 6) {
                        String spotter = parts[1];
                        double enemyX = Double.parseDouble(parts[4]);
                        double enemyY = Double.parseDouble(parts[5]);
                        stepsSinceEnemyUpdate = 0;

                        applyFormationOffset(spotter, enemyX, enemyY);
                    }
                } catch (Exception ignored) {}
            }
        }
    }
    private void applyFormationOffset(String spotter, double targetX, double targetY) {
        int spotterPosition = getRolePosition(spotter);
        int myPosition = getRolePosition(robotName);

        int relativeOffset = (myPosition - spotterPosition);

        currentTargetX = targetX + (relativeOffset * FLANK_OFFSET_X);
        currentTargetY = targetY;
    }

    private int getRolePosition(String name) {
        switch (name) {
            case "WARIO": return -1;
            case "MARIO": return 0;
            case "LUIGI": return 1;
            default:      return 0;
        }
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
                targetAngle = normalize(targetAngle + avoidSide * AVOID_STEP);
                return;
            } else {
                commitForwardSteps = 1;
                turnUsesRadarRayCheck = false;
                state = afterTurnState;  // ✅ instead of MOVE
                return;
            }
        }

        state = afterTurnState;          // ✅ instead of MOVE
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
            state = afterTurnState;
            return;
        }

        uTurnChecksLeft--;
        if (uTurnChecksLeft <= 0) {
            // full scan failed: back up more and try moving logic again
            escapeBackSteps = Math.max(escapeBackSteps, 2);
            state = afterTurnState;
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
