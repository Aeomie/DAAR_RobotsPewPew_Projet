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
    private double totalTurned = 0;
    private static final double ANGLEPRECISION = 0.1;

    // --- DETECTION RANGES ---
    private static final int ENEMY_DETECTION_DISTANCE = 150;
    private static final int WRECK_DETECTION_DISTANCE = 150;
    private static final int TEAMMATE_DETECTION_DISTANCE = 150;

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
//            sendLogMessage("SEEKER initialized at (" + (int)myX + "," + (int)myY + ")");
        } else {
            myX = Parameters.teamAMainBot2InitX;
            myY = Parameters.teamAMainBot2InitY;
//            sendLogMessage("BEATER initialized at (" + (int)myX + "," + (int)myY + ")");
        }

        state = State.IDLE;
        isMoving = false;
        oldAngle = myGetHeading();
    }

    @Override
    public void step() {
        updateOdometry();

        switch(state){
            case IDLE:
                state = State.MOVE;
                return;
            case MOVE:
                if(!obstacleCheck()){
                    myMove();
                }else{
                    state = State.TURN_RIGHT;
                    oldAngle = myGetHeading();
                    stepTurn(Parameters.Direction.RIGHT);
                }
                return;
            case TURN_RIGHT:
                if(isSameDirection(myGetHeading(),oldAngle+Parameters.RIGHTTURNFULLANGLE)){
                    state = State.MOVE;
                }else{
                    stepTurn(Parameters.Direction.RIGHT);
                }
                return;
            case TURN_LEFT:
                if(isSameDirection(myGetHeading(),Parameters.NORTH)){
                    state = State.MOVE;
                }else{
                    stepTurn(Parameters.Direction.LEFT);
                }
                return;


        }
    }

    private void updateOdometry(){
        boolean blockedByWall = detectWall();
        boolean blockedByWreck = isBlockedByWreck();
        boolean blockedByTeamMate = isBlockedByTeamMate();
        boolean blockedByOpponent = isBlockedByOpponent();
        if (isMoving){
            if(!blockedByWall && !blockedByWreck && !blockedByTeamMate && !blockedByOpponent){
                myX += Parameters.teamAMainBotSpeed * Math.cos(myGetHeading());
                myY += Parameters.teamAMainBotSpeed * Math.sin(myGetHeading());
            }
            isMoving=false;
        }
        sendLogMessage("State : " + state + " | Position: (" + (int)myX + "," + (int)myY + ") | Heading: " + (int)(myGetHeading()*180/(double)Math.PI) +
                " old angle = " + (int)(oldAngle*180/(double)Math.PI));
    }
    private void myMove(){
        isMoving=true;
        move();
    }
    private boolean isSameDirection(double dir1, double dir2){
        return Math.abs(normalize(dir1)-normalize(dir2))<ANGLEPRECISION;
    }
    private double myGetHeading(){
        double result = getHeading();
        while(result<0) result+=2*Math.PI;
        while(result>2*Math.PI) result-=2*Math.PI;
        return result;
    }
    private double normalize(double dir){
        double res=dir;
        while (res<0) res+=2*Math.PI;
        while (res>=2*Math.PI) res-=2*Math.PI;
        return res;
    }

    /*
    Here you will find All the Detection Functions
    Detect Front Wall
    Detect Radar Osbtacles
    Detect TeamMates
    Detect Opponents
     */

    private boolean isInFront(double objDir){
        return Math.abs(normalize(objDir - myGetHeading())) < Math.PI/6; // 60° cone
    }

    private boolean obstacleCheck(){
        return detectWall() || isBlockedByWreck() || isBlockedByTeamMate() || isBlockedByOpponent();
    }
    private boolean detectWall(){
        return detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
    }
    private boolean isBlockedByWreck(){
        for (IRadarResult o : detectRadar())
            if (o.getObjectType() == IRadarResult.Types.Wreck
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < WRECK_DETECTION_DISTANCE){
                sendLogMessage("Blocked by Wreck at distance: " + o.getObjectDistance());
                return true;
            }
        return false;
    }

    private boolean isBlockedByTeamMate(){
        for (IRadarResult o : detectRadar())
            if ((o.getObjectType() == IRadarResult.Types.TeamMainBot || o.getObjectType() == IRadarResult.Types.TeamSecondaryBot)
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < TEAMMATE_DETECTION_DISTANCE){
                sendLogMessage("Blocked by Teammate at distance: " + o.getObjectDistance());
                return true;
            }
        return false;
    }

    private boolean isBlockedByOpponent(){
        for (IRadarResult o : detectRadar())
            if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot)
                    && isInFront(o.getObjectDirection())
                    && o.getObjectDistance() < ENEMY_DETECTION_DISTANCE){
                sendLogMessage("Blocked by Opponent at distance: " + o.getObjectDistance());
                return true;
            }
        return false;
    }


}
