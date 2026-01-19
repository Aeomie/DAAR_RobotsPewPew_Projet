package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

public class Robot extends Brain {

    private enum Role { BEATER, SEEKER, UNDEFINED }
    private enum State {
        TURN_LEFT, MOVE, TURN_RIGHT, U_TURN_LEFT, U_TURN_RIGHT, IDLE
    }
    //---VARIABLES---//
    private Role role = Role.UNDEFINED;
    private State state = State.IDLE;
    private double myX, myY;
    private double oldAngle;
    private boolean isMoving;
    private static final double ANGLEPRECISION = 0.1;

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
        updateOdometry();

        switch(state){
            case IDLE:
                state = state.MOVE;
                return;
            case MOVE:
                if(!detectWall()){
                    myMove();
                }else{
                    state = State.TURN_RIGHT;
                    oldAngle = getHeading();
                    stepTurn(Parameters.Direction.RIGHT);
                }
                return;
            case TURN_RIGHT:
                if(isSameDirection(getHeading(),oldAngle+Parameters.RIGHTTURNFULLANGLE)){
                    state = State.MOVE;
                    myMove();
                }else{
                    stepTurn(Parameters.Direction.RIGHT);
                }
                return;
            case TURN_LEFT:
                if(isSameDirection(getHeading(),Parameters.NORTH)){
                    state = State.MOVE;
                    myMove();
                }else{
                    stepTurn(Parameters.Direction.LEFT);
                }
                return;


        }
    }

    private void updateOdometry(){
        if (isMoving){
            myX+=Parameters.teamAMainBotSpeed*Math.cos(getHeading());
            myY+=Parameters.teamAMainBotSpeed*Math.sin(getHeading());
            isMoving=false;
        }
    }
    private boolean detectWall(){
        return detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
    }
    private void myMove(){
        isMoving=true;
        move();
    }
    private boolean isSameDirection(double dir1, double dir2){
        return Math.abs(normalize(dir1)-normalize(dir2))<ANGLEPRECISION;
    }
    private double normalize(double dir){
        double res=dir;
        while (res<0) res+=2*Math.PI;
        while (res>=2*Math.PI) res-=2*Math.PI;
        return res;
    }
}
