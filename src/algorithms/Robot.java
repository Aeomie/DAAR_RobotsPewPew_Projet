package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;

public class Robot extends Brain {

    private enum Role { WARIO, MARIO, LUIGI, UNDEFINED }
    private enum State {
        MOVE, TURNING, BACKING_UP, WAITING, ATTACKING, CONVERGING, IDLE, STOPPED,
        AVOID_BACK, AVOID_TURN, AVOID_FORWARD
    }

    // --- VARIABLES ---
    private Role role = Role.UNDEFINED;
    private String robotName = "Unknown";
    private State state = State.IDLE;

    private double myX, myY;
    private double targetAngle;

    // --- ODOMETRY MOVE FLAG ---
    // +1 forward, -1 backward, 0 none
    private int moveSign = 0;

    private int backupSteps = 0;
    private int consecutiveBlocked = 0;
    private int waitCounter = 0;

    private static final double ANGLE_PRECISION = 0.1;
    private static final int MAX_BACKUP_STEPS = 3;
    private static final int MAX_WAIT_TIME = 5;

    // ✅ SMALL TURN LIKE SECONDARY
    private static final double TURN_INCREMENT = Math.PI / 6; // 30°

    // --- TACTICAL POSITIONING ---
    private static final double FLANK_OFFSET_X = 150;
    private static final double FLANK_OFFSET_Y = 100;

    // --- SHARED ENEMY TRACKING ---
    private double sharedEnemyX = -1;
    private double sharedEnemyY = -1;
    private int stepsSinceEnemyUpdate = 0;
    private static final int ENEMY_INFO_EXPIRY = 20;

    // --- DETECTION RANGES ---
    private static final int ENEMY_DETECTION_DISTANCE = 400;
    private static final int WRECK_DETECTION_DISTANCE = 100;
    private static final int TEAMMATE_DETECTION_DISTANCE = 120;
    private static final int SECONDARY_BOT_DETECTION_DISTANCE = 80;

    // STOPPED TIME
    private static final int STOPPED_TIME = 5000;
    private int noEnemySignalCooldown = 0;

    // Map bounds discovered (kept for your existing comms)
    private double northBound = -1;
    private double westBound = -1;
    private double eastBound = -1;
    private double southBound = -1;

    // --- AVOIDANCE MEMORY ---
    private State resumeState = State.MOVE;     // where to go back after avoiding
    private double resumeX = -1, resumeY = -1;  // goal to continue (for converging)
    private double resumePrecision = 100;

    private int avoidBackSteps = 0;
    private int avoidForwardSteps = 0;

    private static final int AVOID_BACK_STEPS = 5;      // tweak 3–8
    private static final int AVOID_FORWARD_STEPS = 10;  // tweak 6–15

    @Override
    public void activate() {
        boolean seesNorth = false;
        boolean seesSouth = false;

        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.TeamMainBot) {
                if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) seesNorth = true;
                if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH)) seesSouth = true;
            }
        }

        int whoAmI;
        if (seesNorth && !seesSouth) {
            whoAmI = 3; // bottom -> LUIGI
        } else if (seesSouth && !seesNorth) {
            whoAmI = 1; // top -> WARIO
        } else {
            whoAmI = 2; // middle -> MARIO
        }

        if (whoAmI == 1) {
            myX = Parameters.teamAMainBot1InitX;
            myY = Parameters.teamAMainBot1InitY;
            robotName = "WARIO";
            role = Role.WARIO;
        }
        if (whoAmI == 2) {
            myX = Parameters.teamAMainBot2InitX;
            myY = Parameters.teamAMainBot2InitY;
            robotName = "MARIO";
            role = Role.MARIO;
        }
        if (whoAmI == 3) {
            myX = Parameters.teamAMainBot3InitX;
            myY = Parameters.teamAMainBot3InitY;
            robotName = "LUIGI";
            role = Role.LUIGI;
        }

        sendLogMessage("=== I AM " + robotName + "! ===");

        state = State.STOPPED;

        moveSign = 0;
        targetAngle = myGetHeading();
        noEnemySignalCooldown = STOPPED_TIME;

        // reset avoid memory
        resumeState = State.MOVE;
        resumeX = resumeY = -1;
        resumePrecision = 100;
        avoidBackSteps = avoidForwardSteps = 0;
        backupSteps = 0;
        consecutiveBlocked = 0;
        waitCounter = 0;
    }

    @Override
    public void step() {
        updateOdometry();
        readTeammateMessages();

        // if sees enemy, attack
        if (enemyCheck() && state != State.ATTACKING) {
            state = State.ATTACKING;
        }

        // expire shared enemy info
        stepsSinceEnemyUpdate++;
        if (stepsSinceEnemyUpdate > ENEMY_INFO_EXPIRY) {
            sharedEnemyX = -1;
            sharedEnemyY = -1;
            if (state == State.CONVERGING) state = State.MOVE;
        }

        noEnemySignalCooldown = Math.max(0, noEnemySignalCooldown - 1);

        switch (state) {
            case STOPPED:
                if (sharedEnemyX != -1) { state = State.CONVERGING; break; }
                if (enemyCheck()) { state = State.ATTACKING; break; }
                if (noEnemySignalCooldown == 0) state = State.MOVE;
                break;

            case IDLE:
                state = State.MOVE;
                consecutiveBlocked = 0;
                break;

            case MOVE:
                // teammate found enemy -> converge
                if (sharedEnemyX != -1 && !enemyCheck()) {
                    state = State.CONVERGING;
                    break;
                }

                if (enemyCheck()) { state = State.ATTACKING; break; }

                if (!obstacleCheck()) {
                    myMove();
                    consecutiveBlocked = 0;
                } else {
                    consecutiveBlocked++;

                    // global avoidance: don't bump-loop
                    startAvoiding(State.MOVE, -1, -1, 0);
                }
                break;

            case CONVERGING:
                if (sharedEnemyX != -1) {
                    meetAtPoint(sharedEnemyX, sharedEnemyY, 100);
                } else {
                    state = State.MOVE;
                }
                if (enemyCheck()) state = State.ATTACKING;
                break;

            case ATTACKING:
                if (enemyCheck()) shootEnemy();
                else state = State.MOVE;
                break;

            case WAITING:
                waitCounter++;
                if (waitCounter >= MAX_WAIT_TIME) {
                    state = State.BACKING_UP;
                    backupSteps = 0;
                    waitCounter = 0;
                } else if (!isBlockedByTeamMainBotOnly()) {
                    state = State.MOVE;
                    waitCounter = 0;
                }
                break;

            // keep your old backup for "generic" (still useful sometimes)
            case BACKING_UP:
                if (backupSteps < MAX_BACKUP_STEPS) {
                    myMoveBack();
                    backupSteps++;
                } else {
                    state = State.TURNING;
                    targetAngle = calculateAvoidanceAngleSmall();
                }
                break;

            case TURNING:
                if (isSameDirection(myGetHeading(), targetAngle)) {
                    state = State.MOVE;
                } else {
                    Parameters.Direction turnDir = getTurnDirection(myGetHeading(), targetAngle);
                    stepTurn(turnDir);
                }
                break;

            // ===== NEW AVOIDANCE MACHINE (SIDESTEP) =====
            case AVOID_BACK:
                if (avoidBackSteps > 0) {
                    myMoveBack();
                    avoidBackSteps--;
                } else {
                    targetAngle = calculateAvoidanceAngleSmall();
                    state = State.AVOID_TURN;
                }
                break;

            case AVOID_TURN:
                if (isSameDirection(myGetHeading(), targetAngle)) {
                    state = State.AVOID_FORWARD;
                } else {
                    Parameters.Direction d = getTurnDirection(myGetHeading(), targetAngle);
                    stepTurn(d);
                }
                break;

            case AVOID_FORWARD:
                if (avoidForwardSteps > 0) {
                    if (!obstacleCheck()) {
                        myMove();
                        avoidForwardSteps--;
                    } else {
                        // still blocked: re-avoid (but keep resume goal)
                        avoidBackSteps = AVOID_BACK_STEPS;
                        state = State.AVOID_BACK;
                    }
                } else {
                    // resume mission
                    if (resumeState == State.CONVERGING) {
                        state = State.CONVERGING;
                    } else {
                        state = State.MOVE;
                    }
                }
                break;
        }
    }

    // === AVOIDANCE MEMORY ===

    private void startAvoiding(State comeBackTo, double gx, double gy, double prec) {
        resumeState = comeBackTo;
        resumeX = gx;
        resumeY = gy;
        resumePrecision = prec;

        avoidBackSteps = AVOID_BACK_STEPS;
        avoidForwardSteps = AVOID_FORWARD_STEPS;

        state = State.AVOID_BACK;
    }

    // === NAVIGATION METHODS ===

    private void meetAtPoint(double x, double y, double precision) {
        double distance = Math.hypot(x - myX, y - myY);

        if (distance < precision) {
            sendLogMessage(robotName + " >>> Arrived near target!");
            // stay in converging logic but can fall back to MOVE if you want
            state = State.MOVE;
            return;
        }

        double angleToTarget = Math.atan2(y - myY, x - myX);

        if (!isSameDirection(myGetHeading(), angleToTarget)) {
            Parameters.Direction dir = getTurnDirection(myGetHeading(), angleToTarget);
            stepTurn(dir);
            return;
        }

        if (obstacleCheck()) {
            // IMPORTANT: avoid but keep goal
            startAvoiding(State.CONVERGING, x, y, precision);
            return;
        }

        myMove();
    }

    // ✅ SMALL-TURN AVOIDANCE (30° targets)
    private double calculateAvoidanceAngleSmall() {
        double heading = myGetHeading();
        int leftObs = 0, rightObs = 0;

        for (IRadarResult o : detectRadar()) {
            if (isBehind(o.getObjectDirection())) continue;
            if (o.getObjectDistance() <= 250) {
                double rel = normalize(o.getObjectDirection() - heading);
                if (rel > 0 && rel < Math.PI) rightObs++;
                else leftObs++;
            }
        }

        // pick the less crowded side
        if (leftObs < rightObs) return normalize(heading - TURN_INCREMENT);
        if (rightObs < leftObs) return normalize(heading + TURN_INCREMENT);

        // tie-breaker: alternate using consecutiveBlocked
        if (consecutiveBlocked % 2 == 0) return normalize(heading + TURN_INCREMENT);
        return normalize(heading - TURN_INCREMENT);
    }

    private Parameters.Direction getTurnDirection(double current, double target) {
        double diff = normalize(target - current);
        return (diff < Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
    }

    // === ODOMETRY + MOVE WRAPPERS ===

    private void updateOdometry() {
        boolean blockedByWall = detectWall();
        boolean blockedByWreck = isBlockedByWreckObstacle();
        boolean blockedByTeamMate = isBlockedByTeamMate();
        boolean blockedByOpponent = isBlockedByOpponent();

        if (moveSign != 0) {
            // if moving forward and blocked, don't integrate.
            // if moving back, we ignore "front wall" sensor; still can be blocked by bots/wrecks though
            boolean blocked = (blockedByWreck || blockedByTeamMate || blockedByOpponent || (moveSign > 0 && blockedByWall));

            if (!blocked) {
                double step = Parameters.teamAMainBotSpeed * moveSign;
                myX += step * Math.cos(myGetHeading());
                myY += step * Math.sin(myGetHeading());
            }

            moveSign = 0;
        }
    }

    private void myMove() {
        moveSign = +1;
        move();
    }

    private void myMoveBack() {
        moveSign = -1;
        moveBack();
    }

    // === COMBAT METHODS ===

    private void shootEnemy() {
        ArrayList<IRadarResult> enemiesShooters = new ArrayList<>();
        ArrayList<IRadarResult> enemiesScouts = new ArrayList<>();

        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) &&
                    o.getObjectDistance() <= ENEMY_DETECTION_DISTANCE) {

                if (isBlockedByWreck(o.getObjectDirection(), o.getObjectDistance())) continue;

                if (o.getObjectType() == IRadarResult.Types.OpponentMainBot) enemiesShooters.add(o);
                else enemiesScouts.add(o);
            }
        }

        if (!enemiesShooters.isEmpty()) {
            IRadarResult target = enemiesShooters.get(0);
            for (IRadarResult e : enemiesShooters)
                if (e.getObjectDistance() < target.getObjectDistance()) target = e;

            fire(target.getObjectDirection());
            broadcastEnemyPosition(target);

        } else if (!enemiesScouts.isEmpty()) {
            IRadarResult target = enemiesScouts.get(0);
            for (IRadarResult e : enemiesScouts)
                if (e.getObjectDistance() < target.getObjectDistance()) target = e;

            fire(target.getObjectDirection());
            broadcastEnemyPosition(target);
        }
    }

    private boolean enemyCheck() {
        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) &&
                    o.getObjectDistance() <= ENEMY_DETECTION_DISTANCE) {

                if (isBlockedByWreck(o.getObjectDirection(), o.getObjectDistance())) continue;
                return true;
            }
        }
        return false;
    }

    private boolean isBlockedByWreck(double enemyDirection, double enemyDistance) {
        double botRadius = Parameters.teamAMainBotRadius;

        for (IRadarResult wreck : detectRadar()) {
            if (wreck.getObjectType() == IRadarResult.Types.Wreck &&
                    wreck.getObjectDistance() < enemyDistance) {

                double angularWidth = Math.atan(botRadius / wreck.getObjectDistance());
                double angleDiff = Math.abs(normalize(wreck.getObjectDirection() - enemyDirection));
                if (angleDiff > Math.PI) angleDiff = 2 * Math.PI - angleDiff;

                if (angleDiff <= angularWidth) return true;
            }
        }
        return false;
    }

    // === COMMUNICATION METHODS ===

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

        sharedEnemyX = targetX + (relativeOffset * FLANK_OFFSET_X);
        sharedEnemyY = targetY;
    }

    private int getRolePosition(String name) {
        switch (name) {
            case "WARIO": return -1;
            case "MARIO": return 0;
            case "LUIGI": return 1;
            default:      return 0;
        }
    }

    // === HELPERS ===

    private boolean isSameDirection(double dir1, double dir2) {
        return Math.abs(normalize(dir1) - normalize(dir2)) < ANGLE_PRECISION;
    }

    private double myGetHeading() {
        double result = getHeading();
        while (result < 0) result += 2 * Math.PI;
        while (result >= 2 * Math.PI) result -= 2 * Math.PI;
        return result;
    }

    private double normalize(double dir) {
        double res = dir;
        while (res < 0) res += 2 * Math.PI;
        while (res >= 2 * Math.PI) res -= 2 * Math.PI;
        return res;
    }

    // ===== DETECTION FUNCTIONS =====

    private boolean isInFront(double objDir) {
        double relativeAngle = normalize(objDir - myGetHeading());
        return relativeAngle < Math.PI / 4 || relativeAngle > (2 * Math.PI - Math.PI / 4);
    }

    private boolean isBehind(double objDir) {
        double relativeAngle = normalize(objDir - myGetHeading());
        return relativeAngle > (3 * Math.PI / 4) && relativeAngle < (5 * Math.PI / 4);
    }

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

    private boolean detectWall() {
        return detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
    }

    private boolean isBlockedByWreckObstacle() {
        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.Wreck
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < WRECK_DETECTION_DISTANCE) {
                return true;
            }
        }
        return false;
    }

    private boolean isBlockedByTeamMainBot() {
        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.TeamMainBot
                    && isInFront(o.getObjectDirection())
                    && !isBehind(o.getObjectDirection())
                    && o.getObjectDistance() < TEAMMATE_DETECTION_DISTANCE) {
                return true;
            }
        }
        return false;
    }

    private boolean isBlockedByTeamMate() {
        return isBlockedByTeamMainBot() || isBlockedBySecondaryBot();
    }

    private boolean isBlockedBySecondaryBot() {
        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.TeamSecondaryBot
                    || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot)
                    && isInFront(o.getObjectDirection())
                    && !isBehind(o.getObjectDirection())
                    && o.getObjectDistance() < SECONDARY_BOT_DETECTION_DISTANCE) {
                return true;
            }
        }
        return false;
    }

    private boolean isBlockedByOpponent() {
        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot
                    || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot)
                    && isInFront(o.getObjectDirection())
                    && !isBehind(o.getObjectDirection())
                    && o.getObjectDistance() < ENEMY_DETECTION_DISTANCE) {
                return true;
            }
        }
        return false;
    }
}
