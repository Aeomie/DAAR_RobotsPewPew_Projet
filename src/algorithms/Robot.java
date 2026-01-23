package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;

public class Robot extends Brain {

    private enum Role { WARIO, MARIO, LUIGI, UNDEFINED }
    private enum State {
        MOVE, TURNING, BACKING_UP, WAITING, ATTACKING, CONVERGING, IDLE, STOPPED
    }

    //---VARIABLES---//
    private Role role = Role.UNDEFINED;
    private String robotName = "Unknown";
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

    // --- TACTICAL POSITIONING ---
    private static final double FLANK_OFFSET_X = 150;  // Horizontal flanking distance
    private static final double FLANK_OFFSET_Y = 100;  // Vertical positioning offset
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
    private static final int STOPPED_TIME = 2000;
    private int noEnemySignalCooldown = 0;

    // Map bounds discovered
    private double northBound = -1;
    private double westBound = -1;
    private double eastBound = -1;
    private double southBound = -1;

    @Override
    public void activate() {
        // Determine role based on initial position
        // Bot 1 (left side) = WARIO
        // Bot 2 (center/right) = MARIO or LUIGI based on radar detection

        int botsAbove = 0;  // Teammates to the north
        int botsBelow = 0;  // Teammates to the south

        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.TeamMainBot) {
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
        if(botsBelow >= botsAbove) {
            if(botsBelow == botsAbove) {
                // Mid bot
                role = Role.MARIO;
                robotName = "MARIO";
                myX = Parameters.teamAMainBot2InitX;
                myY = Parameters.teamAMainBot2InitY;

            }else{
                // Top Bot
                role = Role.WARIO;
                robotName = "WARIO";
                myX = Parameters.teamAMainBot1InitX;
                myY = Parameters.teamAMainBot1InitY;

            }
        }else{
            // Bottom Bot
            role = Role.LUIGI;
            robotName = "LUIGI";
            myX = Parameters.teamAMainBot3InitX;
            myY = Parameters.teamAMainBot3InitY;

        }
        sendLogMessage("=== I AM " + robotName + "! ===");
//        state = State.IDLE;
        state = State.STOPPED;
        isMoving = false;
        targetAngle = myGetHeading();
        noEnemySignalCooldown = STOPPED_TIME;
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
        noEnemySignalCooldown = Math.max(0, noEnemySignalCooldown - 1);
        switch(state) {
            case STOPPED:
                // If we received enemy info from a teammate -> react immediately
                if (sharedEnemyX != -1) {
                    state = State.CONVERGING;
                    break;
                }

                // Passive defense
                if (enemyCheck()) {
                    state = State.ATTACKING;
                    break;
                }

                // Timeout -> resume exploration
                if (noEnemySignalCooldown == 0) {
                    state = State.MOVE;
                }
                break;

            case IDLE:
                state = State.MOVE;
                consecutiveBlocked = 0;
                break;

            case MOVE:
                if (sharedEnemyX != -1 && !enemyCheck()) {
                    sendLogMessage(robotName + ": Teammate spotted enemy! Converging on position.");
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
            sendLogMessage(robotName + " firing at enemy main bot at " + (int)target.getObjectDistance() + "mm");
            broadcastEnemyPosition(target);

        } else if (!enemiesScouts.isEmpty()) {
            IRadarResult target = enemiesScouts.get(0);
            for (IRadarResult enemy : enemiesScouts) {
                if (enemy.getObjectDistance() < target.getObjectDistance()) {
                    target = enemy;
                }
            }
            fire(target.getObjectDirection());
            sendLogMessage(robotName + " firing at enemy secondary bot at " + (int)target.getObjectDistance() + "mm");
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

        // Broadcast both spotter position AND enemy position for smart convergence
        String message = "ENEMY_LOCATION|" + robotName + "|" +
                (int)myX + "|" + (int)myY + "|" +
                (int)enemyAbsoluteX + "|" + (int)enemyAbsoluteY;
        broadcast(message);
        sendLogMessage(robotName + " broadcasting: I'm at (" + (int)myX + "," + (int)myY +
                "), enemy at (" + (int)enemyAbsoluteX + "," + (int)enemyAbsoluteY + ")");
    }

    private void readTeammateMessages() {
        ArrayList<String> messages = fetchAllMessages();

        for (String msg : messages) {
            if (msg.startsWith("ENEMY_LOCATION|")) {
                try {
                    String[] parts = msg.split("\\|");
                    if (parts.length == 6) {
                        String spotter = parts[1];

                        // Ignore our own messages
                        if (robotName.equals(spotter)) {
                            continue;
                        }

                        double spotterX = Double.parseDouble(parts[2]);
                        double spotterY = Double.parseDouble(parts[3]);
                        double enemyX = Double.parseDouble(parts[4]);
                        double enemyY = Double.parseDouble(parts[5]);
                        stepsSinceEnemyUpdate = 0;


                        // Form up around the spotter's position
                        applyFormationOffset(spotter, enemyX, enemyY);

                        sendLogMessage(robotName + " received from " + spotter +
                                " - Converging to (" + (int)sharedEnemyX + "," + (int)sharedEnemyY + ")");
                    }
                } catch (Exception e) {
                    // Invalid message, ignore
                }
            }
            if (msg.startsWith("BORDER")){
                try{
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
                }catch (Exception e) {
                    // Invalid message, ignore
                }
            }
            if(msg.startsWith("SCOUT_ENEMY_LOCATION")) {
                try {
                    String[] parts = msg.split("\\|");
                    if (parts.length == 6) {
                        String spotter = parts[1];


                        double spotterX = Double.parseDouble(parts[2]);
                        double spotterY = Double.parseDouble(parts[3]);
                        double enemyX = Double.parseDouble(parts[4]);
                        double enemyY = Double.parseDouble(parts[5]);
                        stepsSinceEnemyUpdate = 0;

                        applyFormationOffset(spotter, enemyX, enemyY);
                        sendLogMessage(robotName + " received from " + spotter +
                                " - Converging to (" + (int)sharedEnemyX + "," + (int)sharedEnemyY + ")");
                    }
                }catch (Exception e) {
                    // Invalid message, ignore
                }
            }

        }
    }

    private void applyFormationOffset(String spotter, double targetX, double targetY) {
        // Form up around the spotter maintaining formation order
        // Calculate relative position: WARIO=-1, MARIO=0, LUIGI=1

        int spotterPosition = getRolePosition(spotter);
        int myPosition = getRolePosition(robotName);

        // Calculate offset: difference in positions * FLANK_OFFSET_X
        int relativeOffset = (myPosition - spotterPosition);

        sharedEnemyX = targetX + (relativeOffset * FLANK_OFFSET_X);
        sharedEnemyY = targetY;

        sendLogMessage(robotName + ": Forming at offset " + relativeOffset + " from " + spotter);
    }

    private int getRolePosition(String name) {
        // Return numeric position: WARIO=-1, MARIO=0, LUIGI=1
        switch (name) {
            case "WARIO":
                return -1;
            case "MARIO":
                return 0;
            case "LUIGI":
                return 1;
            default:
                return 0;
        }
    }


    // === NAVIGATION METHODS === //

    private void meetAtPoint(double x, double y, double precision) {
        double distance = Math.hypot(x - myX, y - myY);

        if (distance < precision) {
            sendLogMessage(robotName + " >>> Arrived at enemy location!");
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
            sendLogMessage(robotName + " >>> Converging on enemy. Distance: " + (int)distance + "mm");
        } else {
            state = State.BACKING_UP;
            backupSteps = 0;
        }
    }

    private void handleTeammateEncounter() {
        // Priority based on character roles
        // MARIO (center) has highest priority, WARIO and LUIGI yield
        if (role == Role.MARIO) {
            // Mario waits briefly
            state = State.WAITING;
            waitCounter = 0;
            sendLogMessage("MARIO: Waiting for teammate to pass");
        } else {
            // Wario and Luigi go around immediately
            state = State.BACKING_UP;
            backupSteps = 0;
            consecutiveBlocked++;
            sendLogMessage(robotName + ": Going around teammate");
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