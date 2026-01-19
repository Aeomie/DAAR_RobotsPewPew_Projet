package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;

public class RobotScript extends Brain {

    private static final double ANGLEPRECISION = 0.1;

    private enum Role { BEATER, SEEKER, UNDEFINED }
    private enum State {
        TURN_LEFT, MOVE, TURN_RIGHT, U_TURN_LEFT, U_TURN_RIGHT, U_TURN_LEFT_AGAIN, U_TURN_RIGHT_AGAIN, SINK, IDLE, MEET_POINT, ATTACK_ENEMY
    }
    //---VARIABLES---//
    private Role role = Role.UNDEFINED;
    private State state = State.IDLE;
    private double myX, myY;
    private double oldAngle;
    private boolean isMoving;

    private static final int MOVES_BEFORE_MEETING = 50;
    private int moveCounter = 0;
    private double meetPointX = 1500;
    private double meetPointY = 1500;
    private boolean is_Going_MeetPoint = false;
    private boolean meetingPointCompleted = false;

    // Wall distance tracking variables
    private boolean wallDetected = false;
    private double wallDetectionX = 0;
    private double wallDetectionY = 0;
    private double wallAvoidanceDistance = 100;
    // U-turn direction toggle
    private boolean useLeftUturn = false;

    // -- ENEMY VARIABLES -- //
    private static final int ENEMY_DETECTION_DISTANCE = 400;
    private static final int WRECK_DETECTION_DISTANCE = 400;

    // -- ODOMETRY TRACKING -- //
    private static final int ARENA_WIDTH = 3000;
    private static final int ARENA_HEIGHT = 2000;

    public RobotScript() { super(); }

    @Override
    public void activate() {
        role = Role.SEEKER;
        for (IRadarResult o : detectRadar())
            if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) {
                role = Role.BEATER;
                break;
            }

        // Initialize position based on role with CORRECT coordinates
        if (role == Role.SEEKER) {
            myX = Parameters.teamAMainBot1InitX;
            myY = Parameters.teamAMainBot1InitY;
            sendLogMessage("SEEKER initialized at (" + (int)myX + "," + (int)myY + ")");
        } else {
            myX = Parameters.teamAMainBot2InitX;
            myY = Parameters.teamAMainBot2InitY;
            sendLogMessage("BEATER initialized at (" + (int)myX + "," + (int)myY + ")");
        }

        state = State.IDLE;
        isMoving = false;
        oldAngle = getHeading();
    }

    @Override
    public void step() {
        // CRITICAL: Update odometry BEFORE making decisions
        updateOdometryIfMoving();

        // Log position periodically (every 10 steps or so)
        if (Math.random() < 0.1) {
            sendLogMessage(role + " at (" + (int)myX + "," + (int)myY + ") heading " + (int)Math.toDegrees(getHeading()) + "°");
        }

        switch (state) {
            case IDLE:
                sendLogMessage("=== IDLE state: Starting 360° rotation ===");
                state = useLeftUturn ? State.U_TURN_LEFT : State.U_TURN_RIGHT;
                useLeftUturn = !useLeftUturn;
                oldAngle = getHeading();
                return;

            case TURN_LEFT:
                if (!isSameDirection(getHeading(), Parameters.NORTH)) {
                    face(Parameters.NORTH, Parameters.Direction.LEFT);
                    return;
                }
                state = State.MOVE;
                startMove();
                return;

            case MOVE:
                if (moveCounter < MOVES_BEFORE_MEETING) {
                    moveCounter++;
                } else {
                    if (!is_Going_MeetPoint && !meetingPointCompleted) {
                        is_Going_MeetPoint = true;
                    }
                }

                if (obstacleCheck()) {
                    state = State.TURN_RIGHT;
                    oldAngle = getHeading();
                    isMoving = false;
                    return;
                }

                if (enemyCheck()) {
                    sendLogMessage("!!! Enemy found! Switching to ATTACK_ENEMY state !!!");
                    state = State.ATTACK_ENEMY;
                    isMoving = false;
                    return;
                }

                // Handle meeting point navigation
                if (is_Going_MeetPoint) {
                    meetAtPoint(meetPointX, meetPointY, 50);
                    return;
                }

                startMove();
                return;

            case TURN_RIGHT:
                if (!isSameDirection(getHeading(), normalizeAngle(oldAngle + Parameters.RIGHTTURNFULLANGLE))) {
                    stepTurn(Parameters.Direction.RIGHT);
                    return;
                }
                state = State.MOVE;
                startMove();
                return;

            case SINK:
                sendLogMessage("=== SINK state: checking for obstacles ===");
                if (obstacleCheck()) {
                    state = State.TURN_RIGHT;
                    oldAngle = getHeading();
                    isMoving = false;
                    return;
                }
                sendLogMessage("SINK: moving forward");
                startMove();
                return;

            case ATTACK_ENEMY:
                if (enemyCheck()) {
                    shootEnemy();
                    return;
                }

                if (obstacleCheck()) {
                    state = State.TURN_RIGHT;
                    oldAngle = getHeading();
                    return;
                }

                state = State.MOVE;
                return;

            case U_TURN_LEFT:
                if (!isSameDirection(getHeading(), normalizeAngle(oldAngle + Math.PI))) {
                    stepTurn(Parameters.Direction.LEFT);
                    return;
                }
                sendLogMessage("=== U_TURN_LEFT: First 180° complete, continuing to 360° ===");
                state = State.U_TURN_LEFT_AGAIN;
                oldAngle = getHeading();
                return;

            case U_TURN_LEFT_AGAIN:
                if (!isSameDirection(getHeading(), normalizeAngle(oldAngle + Math.PI))) {
                    stepTurn(Parameters.Direction.LEFT);
                    return;
                }
                sendLogMessage("=== U_TURN_LEFT_AGAIN: Full 360° rotation complete! ===");
                state = State.MOVE;
                startMove();
                return;

            case U_TURN_RIGHT:
                if (!isSameDirection(getHeading(), normalizeAngle(oldAngle + Math.PI))) {
                    stepTurn(Parameters.Direction.RIGHT);
                    return;
                }
                sendLogMessage("=== U_TURN_RIGHT: First 180° complete, continuing to 360° ===");
                state = State.U_TURN_RIGHT_AGAIN;
                oldAngle = getHeading();
                return;

            case U_TURN_RIGHT_AGAIN:
                if (!isSameDirection(getHeading(), normalizeAngle(oldAngle + Math.PI))) {
                    stepTurn(Parameters.Direction.RIGHT);
                    return;
                }
                sendLogMessage("=== U_TURN_RIGHT_AGAIN: Full 360° rotation complete! ===");
                state = State.MOVE;
                startMove();
                return;

            default:
                return;
        }
    }

    // === CORE MOVEMENT METHODS === //

    private void startMove() {
        isMoving = true;
        move();
    }

    /**
     * CRITICAL ODOMETRY UPDATE - Only updates position if robot actually moved
     * This prevents position drift when blocked by walls or wrecks
     */
    private void updateOdometryIfMoving() {
        if (!isMoving) return;

        // Check if we're blocked BEFORE updating position
        boolean blockedByWreck = false;
        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.Wreck &&
                    isSameDirection(o.getObjectDirection(), getHeading())) {
                blockedByWreck = true;
                break;
            }
        }

        boolean blockedByWall = (detectFront().getObjectType() == IFrontSensorResult.Types.WALL);

        // ONLY update position if we actually moved
        if (!blockedByWreck && !blockedByWall) {
            double deltaX = Parameters.teamAMainBotSpeed * Math.cos(getHeading());
            double deltaY = Parameters.teamAMainBotSpeed * Math.sin(getHeading());
            myX += deltaX;
            myY += deltaY;

            // Clamp to arena boundaries (optional safety check)
            myX = Math.max(0, Math.min(ARENA_WIDTH, myX));
            myY = Math.max(0, Math.min(ARENA_HEIGHT, myY));
        } else {
            sendLogMessage(">>> Movement blocked! Position NOT updated.");
        }

        isMoving = false;
    }

    private void face(double targetAngle, Parameters.Direction turnDir) {
        double target = normalizeAngle(targetAngle);
        if (!isSameDirection(getHeading(), target)) {
            stepTurn(turnDir);
        }
    }

    private boolean isSameDirection(double dir1, double dir2) {
        return Math.abs(normalizeAngle(dir1) - normalizeAngle(dir2)) < ANGLEPRECISION;
    }

    private double normalizeAngle(double a) {
        double res = a;
        while (res < 0) res += 2 * Math.PI;
        while (res >= 2 * Math.PI) res -= 2 * Math.PI;
        return res;
    }

    // === COMBAT METHODS === //

    private void shootEnemy(){
        ArrayList<IRadarResult> enemiesShooters = new ArrayList<>();
        ArrayList<IRadarResult> enemiesScouts = new ArrayList<>();

        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) &&
                    o.getObjectDistance() <= ENEMY_DETECTION_DISTANCE
            ) {
                if(o.getObjectType() == IRadarResult.Types.OpponentMainBot){
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
            sendLogMessage("Shooting at enemy shooter at " + (int)target.getObjectDistance() + "mm");

        } else if (!enemiesScouts.isEmpty()) {
            IRadarResult target = enemiesScouts.get(0);
            for (IRadarResult enemy : enemiesScouts) {
                if (enemy.getObjectDistance() < target.getObjectDistance()) {
                    target = enemy;
                }
            }
            fire(target.getObjectDirection());
            sendLogMessage("Shooting at enemy scout at " + (int)target.getObjectDistance() + "mm");
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
                double angleDiff = Math.abs(normalizeAngle(wreck.getObjectDirection() - enemyDirection));
                if (angleDiff > Math.PI) angleDiff = 2 * Math.PI - angleDiff;

                if (angleDiff <= angularWidth) {
                    return true;
                }
            }
        }
        return false;
    }

    // === OBSTACLE DETECTION === //

    private boolean obstacleCheck(){
        for (IRadarResult o : detectRadar()) {
            double objDir = o.getObjectDirection();

            if (o.getObjectDistance() <= WRECK_DETECTION_DISTANCE &&
                    isSameDirection(getHeading(), objDir) &&
                    o.getObjectType() == IRadarResult.Types.Wreck){
                return true;
            }
        }

        if (wallDetection()) {
            return true;
        }
        return false;
    }

    private boolean wallDetection() {
        IFrontSensorResult front = detectFront();

        if (front.getObjectType() == IFrontSensorResult.Types.WALL) {
            if (!wallDetected) {
                wallDetected = true;
                wallDetectionX = myX;
                wallDetectionY = myY;
                sendLogMessage(">>> Wall detected at (" + (int)myX + "," + (int)myY + "), starting avoidance.");
                return true;
            }

            double distanceFromDetection = Math.hypot(myX - wallDetectionX, myY - wallDetectionY);
            if (distanceFromDetection >= wallAvoidanceDistance) {
                sendLogMessage(">>> Traveled " + (int)distanceFromDetection + "mm from wall detection point.");
                return true;
            }
            return true;
        }

        if (wallDetected) {
            wallDetected = false;
            sendLogMessage(">>> Wall no longer detected, resuming normal operation.");
        }

        return false;
    }

    // === NAVIGATION === //

    /**
     * Navigate to a specific point using odometry
     * NOW WORKS because we have accurate position tracking!
     */
    private void meetAtPoint(double x, double y, double precision) {
        double distance = Math.hypot(x - myX, y - myY);

        if (distance < precision) {
            sendLogMessage(">>> Arrived at meeting point (" + (int)x + "," + (int)y + ")! Switching to IDLE for U-turn.");
            is_Going_MeetPoint = false;
            meetingPointCompleted = true;
            state = State.IDLE;
            return;
        }

        // Calculate angle to target
        double angleToTarget = Math.atan2(y - myY, x - myX);

        // Turn toward target
        if (!isSameDirection(getHeading(), angleToTarget)) {
            double diff = normalizeAngle(angleToTarget - getHeading());
            Parameters.Direction dir = (diff < Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
            stepTurn(dir);
            return;
        }

        // Move forward when aligned
        if (!obstacleCheck()) {
            startMove();
            sendLogMessage(">>> Moving toward meeting point. Distance: " + (int)distance + "mm");
        }
    }
}