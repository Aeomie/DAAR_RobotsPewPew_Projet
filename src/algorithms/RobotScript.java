package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

public class RobotScript extends Brain {

    private static final double ANGLEPRECISION = 0.1;

    private enum Role { BEATER, SEEKER, UNDEFINED }
    private enum State { TURN_LEFT, MOVE, TURN_RIGHT, U_TURN, SINK, IDLE , MEET_POINT }

    //---VARIABLES---//
    private Role role = Role.UNDEFINED;
    private State state = State.IDLE;
    private double myX, myY;
    private double oldAngle;
    private boolean isMoving;

    public RobotScript() { super(); }

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
            myX = 0;
            myY = 0;
        }

        state = (role == Role.BEATER) ? State.TURN_LEFT : State.SINK;
        isMoving = false;
        oldAngle = getHeading();
    }

    @Override
    public void step() {
        updateOdometryIfMoving();

        if (role == Role.SEEKER) {
            sendLogMessage("Seeker at (" + (int)myX + "," + (int)myY + ") heading " + getHeading());
        }

        // Simple, readable state machine using helper methods
        switch (state) {
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
                IFrontSensorResult front = detectFront();
                if (front.getObjectType() != IFrontSensorResult.Types.WALL) {
                    startMove();
                    return;
                }
                // wall ahead -> prepare right turn, could also be left , whatever
                state = State.TURN_RIGHT;
                oldAngle = getHeading();
                face(oldAngle + Parameters.RIGHTTURNFULLANGLE, Parameters.Direction.RIGHT);
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
                startMove();
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
        // Only update odometry for SEEKER role (mirrors previous behavior)
        if (role == Role.SEEKER) {
            myX += Parameters.teamAMainBotSpeed * Math.cos(getHeading());
            myY += Parameters.teamAMainBotSpeed * Math.sin(getHeading());
        }
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

    private boolean obstacleCheck(){
        IFrontSensorResult front = detectFront();
        if (front.getObjectType() == IFrontSensorResult.Types.WALL) {
            return true;
        }
        for (IRadarResult o : detectRadar()) {
            if (o.getObjectType() == IRadarResult.Types.Wreck &&
                    isSameDirection(o.getObjectDirection(), getHeading())) {
                return true;
            }
        }
        return false;
    }

    private void meetAtPoint(double x, double y, double precision) {
        // Calcule la distance au point cible
        double distance = Math.hypot(myX - x, myY - y);

        // Si pas encore arrivé au point
        if (distance >= precision) {
            // Calcule l'angle vers le point cible
            double angleToTarget = Math.atan2(y - myY, x - myX);

            // Tourne vers le point
            if (!isSameDirection(getHeading(), angleToTarget)) {
                double diff = normalizeAngle(angleToTarget - getHeading());
                Parameters.Direction dir = (diff < Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
                stepTurn(dir);
                return;
            }

            // Avance vers le point
            if (!obstacleCheck()) {
                startMove();
            }
            return;
        }

        // Arrivé au point -> faire un tour complet (360°)
        // Tu devras ajouter une variable pour tracker si le tour est terminé
        stepTurn(Parameters.Direction.RIGHT);
    }

}