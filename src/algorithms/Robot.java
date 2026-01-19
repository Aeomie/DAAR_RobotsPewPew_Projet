package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;

public class Robot extends Brain {

    private enum Role { BEATER, SEEKER, UNDEFINED }
    private enum State {
        MOVE, TURNING, BACKING_UP, WAITING, ATTACKING, IDLE
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

        // Initialize position based on role
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

        // Check for enemies first - highest priority
        if (enemyCheck() && state != State.ATTACKING) {
            state = State.ATTACKING;
        }

        switch(state) {
            case IDLE:
                state = State.MOVE;
                consecutiveBlocked = 0;
                break;

            case MOVE:
                // Check for enemies while moving
                if (enemyCheck()) {
                    state = State.ATTACKING;
                    break;
                }

                if (!obstacleCheck()) {
                    myMove();
                    consecutiveBlocked = 0;
                } else {
                    // Check what type of obstacle
                    if (isBlockedByTeamMainBotOnly()) {
                        // Main teammate blocking - try to coordinate
                        handleTeammateEncounter();
                    } else if (isBlockedBySecondaryBotOnly()) {
                        // Secondary bot blocking - they're usually stationary, go around quickly
                        handleSecondaryBotEncounter();
                    } else {
                        // Wall, wreck, or enemy - avoid normally
                        consecutiveBlocked++;
                        state = State.BACKING_UP;
                        backupSteps = 0;
                    }
                }
                break;

            case ATTACKING:
                if (enemyCheck()) {
                    shootEnemy();
                } else {
                    // No more enemies visible, return to movement
                    state = State.MOVE;
                }
                break;

            case WAITING:
                waitCounter++;
                if (waitCounter >= MAX_WAIT_TIME) {
                    // Waited long enough, now actively avoid
                    state = State.BACKING_UP;
                    backupSteps = 0;
                    waitCounter = 0;
                } else if (!isBlockedByTeamMainBotOnly()) {
                    // Teammate moved, continue
                    state = State.MOVE;
                    waitCounter = 0;
                }
                break;

            case BACKING_UP:
                if (backupSteps < MAX_BACKUP_STEPS) {
                    moveBack();
                    backupSteps++;
                } else {
                    // Done backing up, now turn
                    state = State.TURNING;
                    targetAngle = calculateAvoidanceAngle();
                }
                break;

            case TURNING:
                if (isSameDirection(myGetHeading(), targetAngle)) {
                    // Turn complete - try moving again
                    state = State.MOVE;
                } else {
                    // Continue turning
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

                // Check if wreck is blocking line of sight
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

        // Prioritize main bots (shooters) over secondary bots (scouts)
        if (!enemiesShooters.isEmpty()) {
            IRadarResult target = enemiesShooters.get(0);
            for (IRadarResult enemy : enemiesShooters) {
                if (enemy.getObjectDistance() < target.getObjectDistance()) {
                    target = enemy;
                }
            }
            fire(target.getObjectDirection());
            sendLogMessage("Firing at enemy main bot at " + (int)target.getObjectDistance() + "mm");

        } else if (!enemiesScouts.isEmpty()) {
            IRadarResult target = enemiesScouts.get(0);
            for (IRadarResult enemy : enemiesScouts) {
                if (enemy.getObjectDistance() < target.getObjectDistance()) {
                    target = enemy;
                }
            }
            fire(target.getObjectDirection());
            sendLogMessage("Firing at enemy secondary bot at " + (int)target.getObjectDistance() + "mm");
        }
    }

    private boolean enemyCheck() {
        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) &&
                    o.getObjectDistance() <= ENEMY_DETECTION_DISTANCE) {

                // Don't count enemies blocked by wrecks
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

                // Calculate if wreck blocks the line of sight to enemy
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

    // === NAVIGATION METHODS === //

    private void handleTeammateEncounter() {
        // Use role-based priority: SEEKER has priority over BEATER
        if (role == Role.SEEKER) {
            // Seeker waits briefly to let beater pass
            state = State.WAITING;
            waitCounter = 0;
        } else {
            // Beater tries to go around immediately
            state = State.BACKING_UP;
            backupSteps = 0;
            consecutiveBlocked++;
        }
    }

    private void handleSecondaryBotEncounter() {
        // Secondary bots are usually stationary, so go around them immediately
        // Don't wait - just back up and turn
        state = State.BACKING_UP;
        backupSteps = 0;
        consecutiveBlocked++;
    }

    private double calculateAvoidanceAngle() {
        double currentHeading = myGetHeading();

        // Find best escape direction by checking all obstacles
        boolean leftClear = true;
        boolean rightClear = true;
        int leftObstacles = 0;
        int rightObstacles = 0;

        for (IRadarResult o : detectRadar()) {
            // Ignore obstacles behind us
            if (isBehind(o.getObjectDirection())) {
                continue;
            }

            if (o.getObjectDistance() <= 250) {
                double objDirection = o.getObjectDirection();
                double relativeAngle = normalize(objDirection - currentHeading);

                // Check if obstacle is on left or right
                if (relativeAngle > 0 && relativeAngle < Math.PI) {
                    rightClear = false;
                    rightObstacles++;
                } else {
                    leftClear = false;
                    leftObstacles++;
                }
            }
        }

        // Choose turn direction based on what's clearer
        double turnAmount;
        if (leftClear && !rightClear) {
            turnAmount = Parameters.LEFTTURNFULLANGLE;
        } else if (rightClear && !leftClear) {
            turnAmount = Parameters.RIGHTTURNFULLANGLE;
        } else if (leftObstacles < rightObstacles) {
            turnAmount = Parameters.LEFTTURNFULLANGLE;
        } else if (consecutiveBlocked > 3) {
            // If blocked multiple times, make a larger turn
            turnAmount = Math.PI; // U-turn
        } else {
            // Default: turn right
            turnAmount = Parameters.RIGHTTURNFULLANGLE;
        }

        return normalize(currentHeading + turnAmount);
    }

    private Parameters.Direction getTurnDirection(double current, double target) {
        double diff = normalize(target - current);

        // Turn in the direction that requires less rotation
        if (diff <= Math.PI) {
            return Parameters.Direction.RIGHT;
        } else {
            return Parameters.Direction.LEFT;
        }
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
        // Only consider objects in front (90° cone forward)
        return relativeAngle < Math.PI / 4 || relativeAngle > (2 * Math.PI - Math.PI / 4);
    }

    private boolean isBehind(double objDir) {
        double relativeAngle = normalize(objDir - myGetHeading());
        // Objects behind (90° cone backward)
        return relativeAngle > (3 * Math.PI / 4) && relativeAngle < (5 * Math.PI / 4);
    }

    private boolean obstacleCheck() {
        return detectWall() || isBlockedByWreckObstacle() || isBlockedByTeamMate()
                || isBlockedByOpponent();
    }

    private boolean isBlockedByTeamMainBotOnly() {
        // Returns true ONLY if blocked by main teammate and nothing else
        return isBlockedByTeamMainBot() && !detectWall() && !isBlockedByWreckObstacle()
                && !isBlockedBySecondaryBot() && !isBlockedByOpponent();
    }

    private boolean isBlockedBySecondaryBotOnly() {
        // Returns true ONLY if blocked by secondary bot and nothing else
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