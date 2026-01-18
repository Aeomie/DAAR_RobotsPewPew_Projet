package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class RobotScript extends Brain {

    private static final double ANGLEPRECISION = 0.1;

    private enum Role { BEATER, SEEKER, UNDEFINED }
    private enum State { TURN_LEFT, MOVE, TURN_RIGHT, U_TURN, SINK, IDLE , MEET_POINT, ATTACK_ENEMY }

    //---VARIABLES---//
    private Role role = Role.UNDEFINED;
    private State state = State.IDLE;
    private double myX, myY;
    private double oldAngle;
    private boolean isMoving;

    private static final int MOVES_BEFORE_MEETING = 50;
    private int moveCounter = 0;
    private double meetPointX = 1500; // Example X coordinate for meeting point
    private double meetPointY = 1500; // Example Y coordinate for meeting point
    private boolean is_Going_MeetPoint = false;

    // Wall distance tracking variables
    private boolean wallDetected = false;
    private double wallDetectionX = 0;
    private double wallDetectionY = 0;
    private double wallAvoidanceDistance = 100;


    // -- ENEMY VARIABLES -- //
    private static final int ENEMY_DETECTION_DISTANCE = 400; // in mm
    private static final int WRECK_DETECTION_DISTANCE = 400; // in mm
    public RobotScript() { super(); }

    @Override
    public void activate() {
        role = Role.SEEKER;
        for (IRadarResult o : detectRadar())
            if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) {
                role = Role.BEATER;
                break;
            }

        // Initialize position based on role
        // BEATER is bot2, SEEKER is bot1
        if (role == Role.SEEKER) {
            myX = Parameters.teamAMainBot1InitX;
            myY = Parameters.teamAMainBot1InitY;
        } else {
            // BEATER is the second bot
            myX = Parameters.teamAMainBot2InitX;
            myY = Parameters.teamAMainBot2InitY;
        }

        // Both robots start the same way: IDLE → TURN_LEFT → MOVE
        state = State.IDLE;
        isMoving = false;
        oldAngle = getHeading();
    }

    @Override
    public void step() {
        updateOdometryIfMoving();

//        if (role == Role.SEEKER) {
//            sendLogMessage("Seeker at (" + (int)myX + "," + (int)myY + ") heading " + getHeading());
//        }
//        if (role == Role.BEATER) {
//            sendLogMessage("Beater at (" + (int)myX + "," + (int)myY + ") heading " + getHeading());
//        }

        // Simple, readable state machine using helper methods
        switch (state) {
            case IDLE:
                sendLogMessage("=== IDLE state: switching to TURN_LEFT ===");
                state = State.TURN_LEFT;
                return;
            case TURN_LEFT:
                // Face NORTH
                if (!isSameDirection(getHeading(), Parameters.NORTH)) {
                    face(Parameters.NORTH, Parameters.Direction.LEFT);
                    return;
                }
                state = State.MOVE;
                startMove();
                return;

            case MOVE:
//                if (moveCounter < MOVES_BEFORE_MEETING) {
//                    moveCounter++;
//                } else {
//                    if (!is_Going_MeetPoint) {
//                        is_Going_MeetPoint = true;
//                    }
//                }

                if (obstacleCheck()) {
                    state = State.TURN_RIGHT;
                    oldAngle = getHeading();
                    isMoving = false;
                    return;
                }

                if (enemyCheck()) {
                    sendLogMessage("!!! Enemy found! Switching to ATTACLK_ENEMY state !!!");
                    state = State.ATTACK_ENEMY;
                    isMoving = false;
                    return;
                }

                // Handle meeting point navigation
                if (is_Going_MeetPoint) {
                    meetAtPoint(meetPointX, meetPointY, 50);
                    return; // ← ADD THIS! Prevents startMove() from executing
                }

                // Normal movement
                startMove();
                return;

            case TURN_RIGHT:
                // Continue turning right until reached target angle
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
//                    sendLogMessage("!!! SEEKER hit obstacle! Switching to TURN_RIGHT !!!");
                    state = State.TURN_RIGHT;
                    oldAngle = getHeading();
                    isMoving = false;
                    return;
                }
                sendLogMessage("SINK: moving forward");
                startMove();
                return;
            case MEET_POINT:

            case ATTACK_ENEMY:
                if (enemyCheck()) {
                    shootEnemy();
                    return; // Stay in ATTACK_ENEMY
                }

                // No valid targets - check for obstacles before moving
                if (obstacleCheck()) {
                    state = State.TURN_RIGHT;
                    oldAngle = getHeading();
                    return;
                }

                // Clear to move
                state = State.MOVE;
                return;

            default:
                return;
        }
    }

    // Helpers

    private void startMove() {
        isMoving = true;
        move();
    }

    private void updateOdometryIfMoving() {
        if (!isMoving) return;

        // Update odometry: robot actually moved, so update position
        double deltaX = Parameters.teamAMainBotSpeed * Math.cos(getHeading());
        double deltaY = Parameters.teamAMainBotSpeed * Math.sin(getHeading());
        myX += deltaX;
        myY += deltaY;

//        sendLogMessage("Odometry updated: moved (" + String.format("%.2f", deltaX) + "," + String.format("%.2f", deltaY) + ")");
        isMoving = false;
    }

    /**
     * Rotate one simulation step toward targetAngle using the given direction.
     * If already within precision the method does nothing.
     */
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
            // Shoot at the closest shooter
            IRadarResult target = enemiesShooters.get(0);
            for (IRadarResult enemy : enemiesShooters) {
                if (enemy.getObjectDistance() < target.getObjectDistance()) {
                    target = enemy;
                }
            }
            fire(target.getObjectDirection()); // FIX: fire at direction, not heading
            sendLogMessage("Shooting at enemy shooter at " + (int)target.getObjectDistance() + "mm (health data not available via radar)");

        } else if (!enemiesScouts.isEmpty()) {
            IRadarResult target = enemiesScouts.get(0);
            for (IRadarResult enemy : enemiesScouts) {
                if (enemy.getObjectDistance() < target.getObjectDistance()) {
                    target = enemy;
                }
            }
            fire(target.getObjectDirection());
            sendLogMessage("Shooting at enemy scout at " + (int)target.getObjectDistance() + "mm (health data not available via radar)");
        }
    }


    private boolean obstacleCheck(){
        // Wreck is when a robot dies and leaves debris

        for (IRadarResult o : detectRadar()) {
            double objDir = o.getObjectDirection();

            if (o.getObjectDistance() <= WRECK_DETECTION_DISTANCE & isSameDirection(getHeading(), objDir) &&
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
                // First detection - save position and trigger avoidance
                wallDetected = true;
                wallDetectionX = myX;
                wallDetectionY = myY;
                sendLogMessage(">>> Wall detected at (" + (int)myX + "," + (int)myY + "), starting avoidance.");
                return true;
            }

            // Wall still detected - check if we've traveled far enough
            double distanceFromDetection = Math.hypot(myX - wallDetectionX, myY - wallDetectionY);
            if (distanceFromDetection >= wallAvoidanceDistance) {
                sendLogMessage(">>> Traveled " + (int)distanceFromDetection + "mm from wall detection point.");
                return true; // Keep avoiding
            }
            return true; // Still in avoidance phase
        }

        // Wall no longer in front - safe to reset
        if (wallDetected) {
            wallDetected = false;
            sendLogMessage(">>> Wall no longer detected, resuming normal operation.");
        }

        return false;
    }

    private boolean isBlockedByWreck(double enemyDirection, double enemyDistance) {
        double botRadius = Parameters.teamAMainBotRadius; // or teamBMainBotRadius (all same = 50mm)

        for (IRadarResult wreck : detectRadar()) {
            if (wreck.getObjectType() == IRadarResult.Types.Wreck &&
                    wreck.getObjectDistance() < enemyDistance) {

                // Calculate angular width: bot appears wider when closer
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

    private boolean enemyCheck() {
        for (IRadarResult o : detectRadar()) {
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) &&
                    o.getObjectDistance() <= ENEMY_DETECTION_DISTANCE) {

                // Skip enemies blocked by wrecks
                if (isBlockedByWreck(o.getObjectDirection(), o.getObjectDistance())) {
                    sendLogMessage(">>> Enemy blocked by wreck, ignoring.");
                    continue;
                }

//                sendLogMessage(">>> Live enemy detected at " + (int)o.getObjectDistance() + "mm");
                return true;
            }
        }
        return false;
    }

    private void meetAtPoint(double x, double y, double precision) {
        double distance = Math.hypot(x - myX, y - myY);

        if (distance >= precision) {
            // Calculate angle to target
            double angleToTarget = Math.atan2(y - myY, x - myX);

            // Calculate shortest turn direction
            if (!isSameDirection(getHeading(), angleToTarget)) {
                double diff = normalizeAngle(angleToTarget - getHeading());

                // If difference is less than 180°, turn right; otherwise turn left
                Parameters.Direction dir = (diff < Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
                stepTurn(dir);
                return;
            }

            // Move forward when aligned
            if (!obstacleCheck()) {
                startMove();
            }
            return;
        }

        // Arrived at point - do a full 360° turn
        stepTurn(Parameters.Direction.RIGHT);
    }

}