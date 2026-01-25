package algorithms.old;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;

public class RobotC3 extends Brain {

    private enum Role { WARIO, MARIO, LUIGI, UNDEFINED }
    private enum State {
        MOVE, TURNING, ATTACKING, CONVERGING, IDLE, STOPPED
        // (old AVOID_* / BACKING_UP / WAITING removed from logic)
    }

    // --- ID / STATE ---
    private Role role = Role.UNDEFINED;
    private String robotName = "Unknown";
    private State state = State.IDLE;

    // --- POSITION / HEADING ---
    private double myX, myY;
    private double targetAngle;

    // --- ODOMETRY MOVE FLAG ---
    // +1 forward, -1 backward, 0 none
    private int moveSign = 0;

    private static final double ANGLE_PRECISION = 0.1;

    // --- SIMPLE AVOID (LIKE SECONDARY) ---
    private static final double AVOID_STEP = Math.PI / 12; // 15°
    private int avoidTurns = 0;
    private int avoidSide = 1; // +1 right, -1 left
    private static final int AVOID_GIVEUP_TURNS = 5; // after 5 turns -> "just move"
    private State afterTurnState = State.MOVE;

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

    // --- TACTICAL POSITIONING ---
    private static final double FLANK_OFFSET_X = 150;
    private static final double FLANK_OFFSET_Y = 100;

    // --- WRECK MEMORY (kept) ---
    private static class WreckInfo {
        double x, y;
        WreckInfo(double x, double y) { this.x = x; this.y = y; }
    }
    private final ArrayList<WreckInfo> knownWrecks = new ArrayList<>();
    private static final double WRECK_SAME_POS_EPS = 25.0;

    @Override
    public void activate() {
        identifyRole();
        sendLogMessage("=== I AM " + robotName + "! ===");

        state = State.STOPPED;
        moveSign = 0;
        targetAngle = myGetHeading();
        noEnemySignalCooldown = STOPPED_TIME;

        avoidTurns = 0;
        avoidSide = 1;

        knownWrecks.clear();
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
        }
    }

    @Override
    public void step() {
        updateOdometry();
        rememberWrecks();
        readTeammateMessages();

        // refresh attack state if sees enemy
        if (enemyCheck() && state != State.ATTACKING) state = State.ATTACKING;

        // expire shared enemy
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
                break;

            case ATTACKING:
                if (enemyCheck()) shootEnemy();
                else state = State.MOVE;
                break;

            case CONVERGING:
                if (sharedEnemyX != -1) {
                    meetAtPoint(sharedEnemyX, sharedEnemyY, 100);
                } else {
                    state = State.MOVE;
                }
                if (enemyCheck()) state = State.ATTACKING;
                break;

            case MOVE:
                // teammate found enemy -> converge
                if (sharedEnemyX != -1 && !enemyCheck()) {
                    state = State.CONVERGING;
                    break;
                }

                if (enemyCheck()) { state = State.ATTACKING; break; }

                // SIMPLE MOVEMENT (LIKE SECONDARY)
                simpleAvoid(State.MOVE);
                break;

            case TURNING:
                if (!isSameDirection(myGetHeading(), targetAngle)) {
                    stepTurn(getTurnDirection(myGetHeading(), targetAngle));
                } else {
                    state = afterTurnState;
                }
                break;
        }
    }

    // =========================================================
    // SIMPLE AVOIDANCE:
    // - if clear: move
    // - if blocked: turn by small steps
    // - if still blocked after 5 turns: "just move" (we do 1 backstep to break wedge)
    // =========================================================
    private void simpleAvoid(State returnState) {
        if (!obstacleCheck()) {
            myMove();
            avoidTurns = 0;
            return;
        }

        avoidTurns++;

        // after 5 turns of indecision: commit by moving (back 1 tick breaks the squeeze)
        if (avoidTurns >= AVOID_GIVEUP_TURNS) {
            myMoveBack();          // the safest "direct move" when you are wedged
            avoidTurns = 0;
            avoidSide = -avoidSide;
            return;
        }

        // small step turn, alternate sometimes to prevent oscillation
        targetAngle = normalize(myGetHeading() + avoidSide * AVOID_STEP);
        if (avoidTurns % 3 == 0) avoidSide = -avoidSide;

        afterTurnState = returnState;
        state = State.TURNING;
    }

    // =========================================================
    // NAVIGATION
    // =========================================================
    private void meetAtPoint(double x, double y, double precision) {
        double distance = Math.hypot(x - myX, y - myY);
        if (distance < precision) {
            state = State.MOVE;
            return;
        }

        double angleToTarget = Math.atan2(y - myY, x - myX);

        // if need turn toward target
        if (!isSameDirection(myGetHeading(), angleToTarget)) {
            targetAngle = angleToTarget;
            afterTurnState = State.CONVERGING;
            state = State.TURNING;
            return;
        }

        // if blocked, use the same simple avoid, then continue converging
        if (obstacleCheck()) {
            simpleAvoid(State.CONVERGING);
            return;
        }

        myMove();
    }

    private Parameters.Direction getTurnDirection(double current, double target) {
        double diff = normalize(target - current);
        return (diff < Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
    }

    // =========================================================
    // ODOMETRY + MOVE WRAPPERS
    // =========================================================
    private void updateOdometry() {
        boolean blockedByWall = detectWall();
        boolean blockedByWreck = isBlockedByWreckObstacle();
        boolean blockedByTeamMate = isBlockedByTeamMate();
        boolean blockedByOpponent = isBlockedByOpponent();

        if (moveSign != 0) {
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

    // =========================================================
    // COMBAT
    // =========================================================
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

    // =========================================================
    // COMMUNICATION
    // =========================================================
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

    // =========================================================
    // HELPERS
    // =========================================================
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

    // =========================================================
    // DETECTION
    // =========================================================
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

    private boolean isBlockedByTeamMate() {
        return isBlockedByTeamMainBot() || isBlockedBySecondaryBot();
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

    private void rememberWrecks() {
        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() != IRadarResult.Types.Wreck) continue;

            double wx = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
            double wy = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());

            boolean alreadyKnown = false;
            for (WreckInfo w : knownWrecks) {
                if (Math.hypot(w.x - wx, w.y - wy) <= WRECK_SAME_POS_EPS) {
                    alreadyKnown = true;
                    break;
                }
            }

            if (!alreadyKnown) {
                knownWrecks.add(new WreckInfo(wx, wy));
            }
        }
    }
}
