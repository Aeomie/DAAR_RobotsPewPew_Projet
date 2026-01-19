package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;

public class Robot extends Brain {

    private enum Role { BEATER, SEEKER, UNDEFINED }
    private enum State {
        MOVE, TURNING, BACKING_UP, WAITING, ATTACKING, CONVERGING, IDLE
    }

    //---VARIABLES---//
    private Role role = Role.UNDEFINED;
    private State state = State.IDLE;
    private double myX, myY;
    private double targetAngle;
    private boolean isMoving;
    private int backupSteps = 0;
    private int consecutiveBlocked = 0;
    private int waitCounter = 0;
    private static final double ANGLE_PRECISION = 0.1;
    private static final int MAX_BACKUP_STEPS = 3;
    private static final int MAX_WAIT_TIME = 5;

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

    @Override
    public void activate() {
        role = Role.SEEKER;
        for (IRadarResult o : detectRadar())
            if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) {
                role = Role.BEATER;
                break;
            }

        if (role == Role.SEEKER) {
            myX = Parameters.teamAMainBot1InitX;
            myY = Parameters.teamAMainBot1InitY;
        } else {
            myX = Parameters.teamAMainBot2InitX;
            myY = Parameters.teamAMainBot2InitY;
        }

        state = State.IDLE;
        isMoving = false;
        targetAngle = myGetHeading();
    }

    @Override
    public void step() {
        updateOdometry();
        readTeammateMessages();

        if (enemyCheck() && state != State.ATTACKING) {
            state = State.ATTACKING;
        }

        stepsSinceEnemyUpdate++;
        if (stepsSinceEnemyUpdate > ENEMY_INFO_EXPIRY) {
            sharedEnemyX = -1;
            sharedEnemyY = -1;
            if (state == State.CONVERGING) {
                state = State.MOVE;
            }
        }

        switch(state) {
            case IDLE:
                state = State.MOVE;
                consecutiveBlocked = 0;
                break;

            case MOVE:
                if (sharedEnemyX != -1 && !enemyCheck()) {
                    sendLogMessage("Teammate spotted enemy! Converging on position.");
                    state = State.CONVERGING;
                    break;
                }

                if (enemyCheck()) {
                    state = State.ATTACKING;
                    break;
                }

                if (!obstacleCheck()) {
                    myMove();
                    consecutiveBlocked = 0;
                } else {
                    if (isBlockedByTeamMainBotOnly()) {
                        handleTeammateEncounter();
                    } else if (isBlockedBySecondaryBotOnly()) {
                        handleSecondaryBotEncounter();
                    } else {
                        consecutiveBlocked++;
                        state = State.BACKING_UP;
                        backupSteps = 0;
                    }
                }
                break;

            case CONVERGING:
                if (sharedEnemyX != -1) {
                    meetAtPoint(sharedEnemyX, sharedEnemyY, 100);
                } else {
                    state = State.MOVE;
                }

                if (enemyCheck()) {
                    state = State.ATTACKING;
                }
                break;

            case ATTACKING:
                if (enemyCheck()) {
                    shootEnemy();
                } else {
                    state = State.MOVE;
                }
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

            case BACKING_UP:
                if (backupSteps < MAX_BACKUP_STEPS) {
                    moveBack();
                    backupSteps++;
                } else {
                    state = State.TURNING;
                    targetAngle = calculateAvoidanceAngle();
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
        }
    }

    // === COMBAT METHODS === //

    private void shootEnemy() {
        ArrayList<IRadarResult> enemiesShooters = new ArrayList<>();
        ArrayList<IRadarResult> enemiesScouts = new ArrayList<>();

        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) &&
                    o.getObjectDistance() <= ENEMY_DETECTION_DISTANCE) {

                if (isBlockedByWreck(o.getObjectDirection(), o.getObjectDistance())) {
                    continue;
                }

                if (o.getObjectType() == IRadarResult.Types.OpponentMainBot) {
                    enemiesShooters.add(o);
                } else {
                    enemiesScouts.add(o);
                }
            }
        }

        if (!enemiesShooters.isEmpty()) {
            IRadarResult target = enemiesShooters.get(0);
            for (IRadarResult enemy : enemiesShooters) {
                if (enemy.getObjectDistance() < target.getObjectDistance()) {
                    target = enemy;
                }
            }
            fire(target.getObjectDirection());
            sendLogMessage("Firing at enemy main bot at " + (int)target.getObjectDistance() + "mm");
            broadcastEnemyPosition(target);

        } else if (!enemiesScouts.isEmpty()) {
            IRadarResult target = enemiesScouts.get(0);
            for (IRadarResult enemy : enemiesScouts) {
                if (enemy.getObjectDistance() < target.getObjectDistance()) {
                    target = enemy;
                }
            }
            fire(target.getObjectDirection());
            sendLogMessage("Firing at enemy secondary bot at " + (int)target.getObjectDistance() + "mm");
            broadcastEnemyPosition(target);
        }
    }

    private boolean enemyCheck() {
        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) &&
                    o.getObjectDistance() <= ENEMY_DETECTION_DISTANCE) {

                if (isBlockedByWreck(o.getObjectDirection(), o.getObjectDistance())) {
                    continue;
                }

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

                if (angleDiff <= angularWidth) {
                    return true;
                }
            }
        }
        return false;
    }

    // === COMMUNICATION METHODS === //

    private void broadcastEnemyPosition(IRadarResult enemy) {
        double enemyAbsoluteX = myX + enemy.getObjectDistance() * Math.cos(enemy.getObjectDirection());
        double enemyAbsoluteY = myY + enemy.getObjectDistance() * Math.sin(enemy.getObjectDirection());

        String message = "ENEMY_LOCATION|" + (int)enemyAbsoluteX + "|" + (int)enemyAbsoluteY;
        broadcast(message);
        sendLogMessage("Broadcasting enemy at (" + (int)enemyAbsoluteX + "," + (int)enemyAbsoluteY + ")");
    }

    private void readTeammateMessages() {
        ArrayList<String> messages = fetchAllMessages();

        for (String msg : messages) {
            if (msg.startsWith("ENEMY_LOCATION|")) {
                try {
                    String[] parts = msg.split("\\|");
                    if (parts.length == 3) {
                        sharedEnemyX = Double.parseDouble(parts[1]);
                        sharedEnemyY = Double.parseDouble(parts[2]);
                        stepsSinceEnemyUpdate = 0;
                        sendLogMessage("Received enemy location: (" + (int)sharedEnemyX + "," + (int)sharedEnemyY + ")");
                    }
                } catch (Exception e) {
                    // Invalid message, ignore
                }
            }
        }
    }

    // === NAVIGATION METHODS === //

    private void meetAtPoint(double x, double y, double precision) {
        double distance = Math.hypot(x - myX, y - myY);

        if (distance < precision) {
            sendLogMessage(">>> Arrived at enemy location!");
            state = State.MOVE;
            return;
        }

        double angleToTarget = Math.atan2(y - myY, x - myX);

        if (!isSameDirection(myGetHeading(), angleToTarget)) {
            double diff = normalize(angleToTarget - myGetHeading());
            Parameters.Direction dir = (diff < Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
            stepTurn(dir);
            return;
        }

        if (!obstacleCheck()) {
            myMove();
            sendLogMessage(">>> Converging on enemy. Distance: " + (int)distance + "mm");
        } else {
            state = State.BACKING_UP;
            backupSteps = 0;
        }
    }

    private void handleTeammateEncounter() {
        if (role == Role.SEEKER) {
            state = State.WAITING;
            waitCounter = 0;
        } else {
            state = State.BACKING_UP;
            backupSteps = 0;
            consecutiveBlocked++;
        }
    }

    private void handleSecondaryBotEncounter() {
        state = State.BACKING_UP;
        backupSteps = 0;
        consecutiveBlocked++;
    }

    private double calculateAvoidanceAngle() {
        double currentHeading = myGetHeading();
        boolean leftClear = true;
        boolean rightClear = true;
        int leftObstacles = 0;
        int rightObstacles = 0;

        for (IRadarResult o : detectRadar()) {
            if (isBehind(o.getObjectDirection())) {
                continue;
            }

            if (o.getObjectDistance() <= 250) {
                double objDirection = o.getObjectDirection();
                double relativeAngle = normalize(objDirection - currentHeading);

                if (relativeAngle > 0 && relativeAngle < Math.PI) {
                    rightClear = false;
                    rightObstacles++;
                } else {
                    leftClear = false;
                    leftObstacles++;
                }
            }
        }

        double turnAmount;
        if (leftClear && !rightClear) {
            turnAmount = Parameters.LEFTTURNFULLANGLE;
        } else if (rightClear && !leftClear) {
            turnAmount = Parameters.RIGHTTURNFULLANGLE;
        } else if (leftObstacles < rightObstacles) {
            turnAmount = Parameters.LEFTTURNFULLANGLE;
        } else if (consecutiveBlocked > 3) {
            turnAmount = Math.PI;
        } else {
            turnAmount = Parameters.RIGHTTURNFULLANGLE;
        }

        return normalize(currentHeading + turnAmount);
    }

    private Parameters.Direction getTurnDirection(double current, double target) {
        double diff = normalize(target - current);
        return (diff <= Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
    }

    private void updateOdometry() {
        boolean blockedByWall = detectWall();
        boolean blockedByWreck = isBlockedByWreckObstacle();
        boolean blockedByTeamMate = isBlockedByTeamMate();
        boolean blockedByOpponent = isBlockedByOpponent();

        if (isMoving) {
            if (!blockedByWall && !blockedByWreck && !blockedByTeamMate && !blockedByOpponent) {
                myX += Parameters.teamAMainBotSpeed * Math.cos(myGetHeading());
                myY += Parameters.teamAMainBotSpeed * Math.sin(myGetHeading());
            }
            isMoving = false;
        }
    }

    private void myMove() {
        isMoving = true;
        move();
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
        for (IRadarResult o : detectRadar())
            if (o.getObjectType() == IRadarResult.Types.Wreck
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < WRECK_DETECTION_DISTANCE) {
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
}