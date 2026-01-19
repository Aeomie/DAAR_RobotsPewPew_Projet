package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

public class Robot extends Brain {

    private enum Role { BEATER, SEEKER, UNDEFINED }
    private enum State {
        MOVE, TURNING, BACKING_UP, WAITING, IDLE
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
    private static final int ENEMY_DETECTION_DISTANCE = 100;
    private static final int WRECK_DETECTION_DISTANCE = 100;
    private static final int TEAMMATE_DETECTION_DISTANCE = 120;

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

        switch(state) {
            case IDLE:
                state = State.MOVE;
                consecutiveBlocked = 0;
                break;

            case MOVE:
                if (!obstacleCheck()) {
                    myMove();
                    consecutiveBlocked = 0;
                } else {
                    // Check what type of obstacle
                    if (isBlockedByTeamMateOnly()) {
                        // Teammate blocking - try to coordinate
                        handleTeammateEncounter();
                    } else {
                        // Wall, wreck, or enemy - avoid normally
                        consecutiveBlocked++;
                        state = State.BACKING_UP;
                        backupSteps = 0;
                    }
                }
                break;

            case WAITING:
                waitCounter++;
                if (waitCounter >= MAX_WAIT_TIME) {
                    // Waited long enough, now actively avoid
                    state = State.BACKING_UP;
                    backupSteps = 0;
                    waitCounter = 0;
                } else if (!isBlockedByTeamMateOnly()) {
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

    private double calculateAvoidanceAngle() {
        double currentHeading = myGetHeading();

        // Find best escape direction by checking all obstacles
        boolean leftClear = true;
        boolean rightClear = true;
        int leftObstacles = 0;
        int rightObstacles = 0;

        for (IRadarResult o : detectRadar()) {
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
        boolean blockedByWreck = isBlockedByWreck();
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
        return Math.abs(normalize(objDir - myGetHeading())) < Math.PI / 4; // 90° cone
    }

    private boolean obstacleCheck() {
        return detectWall() || isBlockedByWreck() || isBlockedByTeamMate() || isBlockedByOpponent();
    }

    private boolean isBlockedByTeamMateOnly() {
        // Returns true ONLY if blocked by teammate and nothing else
        return isBlockedByTeamMate() && !detectWall() && !isBlockedByWreck() && !isBlockedByOpponent();
    }

    private boolean detectWall() {
        return detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
    }

    private boolean isBlockedByWreck() {
        for (IRadarResult o : detectRadar())
            if (o.getObjectType() == IRadarResult.Types.Wreck
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < WRECK_DETECTION_DISTANCE) {
                return true;
            }
        return false;
    }

    private boolean isBlockedByTeamMate() {
        for (IRadarResult o : detectRadar())
            if ((o.getObjectType() == IRadarResult.Types.TeamMainBot
                    || o.getObjectType() == IRadarResult.Types.TeamSecondaryBot)
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < TEAMMATE_DETECTION_DISTANCE) {
                return true;
            }
        return false;
    }

    private boolean isBlockedByOpponent() {
        for (IRadarResult o : detectRadar())
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot
                    || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot)
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < ENEMY_DETECTION_DISTANCE) {
                return true;
            }
        return false;
    }
}