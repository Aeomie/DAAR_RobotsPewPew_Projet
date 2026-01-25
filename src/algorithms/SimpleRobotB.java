    package algorithms;
    
    import characteristics.IFrontSensorResult;
    import characteristics.IRadarResult;
    import characteristics.Parameters;
    import robotsimulator.Brain;
    
    import java.util.ArrayList;
    
    public class SimpleRobotB extends Brain {
    
        private enum Role { WARIO, MARIO, LUIGI, UNDEFINED }
        private enum State { MOVE, TURNING, BACKING_UP,
            IDLE, UTURN, CONVERGING, TEST_STOPPED, WAITING_FOR_SIGNAL }
    
        private State state = State.MOVE;
    
        // --- turning precision (for "am I aligned yet?") ---
        private static final double ANGLE_PRECISION = 0.05;
    
        // --- radar direction tolerance (for "is obstacle on this ray?") ---
        private static final double RADAR_DIR_EPS = 0.15; // ~8.6°
    
        private double targetAngle = 0;
    
        // --- avoidance steps ---
        private static final double AVOID_STEP = Math.PI / 6; // 30°
        private int avoidSide = 1; // +1 right, -1 left
    
        private int consecutiveBlocks = 0;
        private static final int BLOCK_ESCAPE_TRIGGER = 5;
    
        private int escapeBackSteps = 0;
        private static final int ESCAPE_BACK_STEPS = 3;
    
        // --- wall U-turn scan ---
        private int uTurnChecksLeft = 0;
        private double uTurnStartHeading = 0;
        private int uTurnSide = 1; // +1 right, -1 left
    
        // --- radar as REAL radius bubble (360°) ---
        private static final double RADAR_RADIUS = 150;
    
        // turning reason flags
        private boolean turnUsesRadarRayCheck = false;
    
        // after we found a free heading, commit 1 forward step
        private int commitForwardSteps = 0;
    
        // --- tiny odometry (optional) ---
        private double myX, myY;
        private boolean isMoving = false;
        private boolean lastMoveWasBack = false;
    
        private String robotName = "undefined";
        private Role role = Role.UNDEFINED;
        private State afterTurnState = State.MOVE;
    
        // Map bounds discovered (kept for your existing comms)
        private double northBound = -1;
        private double westBound = -1;
        private double eastBound = -1;
        private double southBound = -1;
    
        // UPDATE ENEMY LOCATION COMMUNICATIONS
        private static final double TARGET_RESET_COOLDOWN = 1000; // start high to avoid immediate use
        private int stepsSinceEnemyUpdate = 0;
        private double currentTargetX = -1;
        private double currentTargetY = -1;
        private enum TargetType { NONE, ENEMY_MAIN, ENEMY_SECONDARY }
        private TargetType currentTargetType = TargetType.NONE;
        private static final double FLANK_OFFSET_X = 150;
        private static final double TARGET_PRECISION = 50;
        private boolean nav_Lock = false; // to differentiate between meetAtPoint to just meet or to enemy location
        private boolean enemy_Lock = false;
        private static final int WAIT_ENEMY_TIME = 80;
        private int enemy_wait_time = -1;
        private static final double FRONT_ATTACK_CONE = 0.35;
    
        private int test_time = 300;
        private int tick = 0;
    
        // WAIT FOR SIGNAL
        private static final int WAIT_FOR_SIGNAL = 500;
        private int wait_signal_time = WAIT_FOR_SIGNAL;
    
        private static final double FLANK_RADIUS = 200;          // stand-off distance from target
        private static final double FLANK_ANGLE  = Math.toRadians(20);
    
        // DEFAULT POINT TO GO BACK TO
        private double defaultX = -1;
        private double defaultY = -1;
        private boolean returningHome = false;
    
        // TARGET REFRESH COOLDOWN
        private static final int TARGET_REFRESH_COOLDOWN = 800;
        private double lastTargetX = -1;
        private double lastTargetY = -1;
        private int targetRefreshCooldown = 0;
        private int target_error_margin = 20;
        private boolean reset_Target = false;
    
        @Override
        public void activate() {
            identifyRole();
    
            state = State.WAITING_FOR_SIGNAL;
            consecutiveBlocks = 0;
            escapeBackSteps = 0;
            avoidSide = 1;
    
            uTurnChecksLeft = 0;
            uTurnStartHeading = 0;
            uTurnSide = 1;
    
            commitForwardSteps = 0;
            turnUsesRadarRayCheck = false;
            targetRefreshCooldown = TARGET_REFRESH_COOLDOWN;
    
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
            defaultX = Parameters.teamBMainBot2InitX;
            defaultY = Parameters.teamBMainBot2InitY;
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
                default:
                    robotName = "UNKNOWN";
                    role = Role.UNDEFINED;
            }
        }
    
        private void dbg(String s) {
            sendLogMessage("[t=" + tick + "] " + s);
        }
    
        private void abandonCurrentTarget() {
            if (lastChanceEnemyCheck() && !reset_Target) return;
    
            reset_Target = false;
            targetRefreshCooldown = TARGET_REFRESH_COOLDOWN;
    
            enemy_Lock = false;
            nav_Lock = false;
            enemy_wait_time = -1;
            stepsSinceEnemyUpdate = 0;
    
            // if already home-ish, just clear target and resume
            if (inRange(myX, defaultX, 50) && inRange(myY, defaultY, 50)) {
                currentTargetX = -1;
                currentTargetY = -1;
                returningHome = false;
                state = State.MOVE;
                return;
            }
    
            currentTargetX = -1;
            currentTargetY = -1;
            returningHome = true;
            goBackToDefaultPosition();
        }
    
    
    
        private void lockTarget(double x , double y, TargetType type) {
            currentTargetX = x;
            currentTargetY = y;
            enemy_Lock = true;
            nav_Lock = true;
            currentTargetType = type;
            state = State.CONVERGING;
        }
        @Override
        public void step() {
            tick++;
            updateOdometry();
            readTeammateMessages();
            // ===== HIGHEST PRIORITY: SHOOT IF POSSIBLE (preempt movement/avoid) =====
            if (tryEngageFrontMainBot()) return;

            IRadarResult e = findClosestEnemyOnRadar();
            if (e != null) {
                double ex = myX + e.getObjectDistance() * Math.cos(e.getObjectDirection());
                double ey = myY + e.getObjectDistance() * Math.sin(e.getObjectDirection());

                // lock/update target fast (even if we were turning/uturning)
                currentTargetX = ex;
                currentTargetY = ey;
                currentTargetType = getEnemyTypeRadar(e);
                enemy_Lock = true;
                nav_Lock = true;
                stepsSinceEnemyUpdate = 0;

                // fire if not team-blocked; otherwise don't enter wiggle-avoid loop here
                double ang = normalize(Math.atan2(currentTargetY - myY, currentTargetX - myX));
                if (!teammateBlocksShot(ang)) {
                    fire(ang);
                    return; // ✅ shooting preempts everything
                }
            }
    //        dbg("state=" + state
    //                + " e_Lock=" + enemy_Lock
    //                + " t_Lock=" + nav_Lock
    //                + " ttl=" + enemy_wait_time
    //                + " target=(" + (int)currentTargetX + "," + (int)currentTargetY + ")");
    
            test_time = Math.max(0, test_time - 1);
            if (currentTargetX != -1 && currentTargetY != -1) {
                if (inRange(lastTargetX, currentTargetX, target_error_margin) &&
                        inRange(lastTargetY, currentTargetY, target_error_margin) &&
                        !reset_Target) {
    
                    targetRefreshCooldown = Math.max(0, targetRefreshCooldown - 1);
                    if (targetRefreshCooldown == 0) reset_Target = true;
    
                } else {
                    lastTargetX = currentTargetX;
                    lastTargetY = currentTargetY;
                    targetRefreshCooldown = TARGET_REFRESH_COOLDOWN;
                    reset_Target = false; // important: because target changed
                }
            }
    
            stepsSinceEnemyUpdate++;
            if ((stepsSinceEnemyUpdate > TARGET_RESET_COOLDOWN &&  !returningHome) || reset_Target) {
                abandonCurrentTarget();
                return;
            }
            // 🆕 SCAN FOR ENEMIES EVERY TICK (if not engaged AND cooldown expired)
            if (!enemy_Lock && !nav_Lock && enemy_wait_time == -1) {
                IRadarResult enemy = findClosestEnemyOnRadar();
                if (enemy != null) {
                    double enemyAbsoluteX = myX + enemy.getObjectDistance() * Math.cos(enemy.getObjectDirection());
                    double enemyAbsoluteY = myY + enemy.getObjectDistance() * Math.sin(enemy.getObjectDirection());
    
                    currentTargetX = enemyAbsoluteX;
                    currentTargetY = enemyAbsoluteY;
                    currentTargetType = getEnemyTypeRadar(enemy);
                    enemy_Lock = true;
                    nav_Lock = true;
                    stepsSinceEnemyUpdate = 0;
    
                    sendLogMessage(robotName + " >>> SPOTTED enemy at (" +
                            (int)enemyAbsoluteX + "," + (int)enemyAbsoluteY + ")!");
                    broadcastEnemyPosition(enemy);
    
                    state = State.CONVERGING;
                    return;
                }
            }
            if (enemy_Lock) {
                dbg("enemy_Lock=true -> trying updateTargetFromRadarIfVisible()");
                boolean seesEnemy = updateTargetFromRadarIfVisible();
                dbg("seesEnemy=" + seesEnemy
                        + " afterUpdate target=(" + (int)currentTargetX + "," + (int)currentTargetY + ")");
    
                if (!seesEnemy) {
                    dbg("LOST enemy -> dropping enemy_Lock, starting TTL");
                    enemy_Lock = false;
                    nav_Lock = true;
                    enemy_wait_time = WAIT_ENEMY_TIME;
                } else {
                    dbg("ABOUT TO SHOOT");
                    boolean fired = shootAtCurrentTarget();
                    if (fired) return;
                    dbg("SHOT CALLED, returning");
                }
            }
    
            switch (state) {
                case WAITING_FOR_SIGNAL:
                    if (nav_Lock && currentTargetX != -1 && currentTargetY != -1) {
                        state = State.CONVERGING;   // or MOVE then it will switch to CONVERGING next tick
                        break;
                    }
    
                    wait_signal_time--;
                    if (wait_signal_time <= 0) state = State.MOVE;
                    break;
                case TEST_STOPPED:
                    if (test_time <= 0) {
                        currentTargetY = 150;
                        currentTargetX = 1500;
                        state = State.CONVERGING;
                    }
                    break;
                case MOVE:
                    if (currentTargetX != -1 && currentTargetY != -1) {
                        state = State.CONVERGING;
                        break;
                    }
                    moveUsingFrontThenRadarRadius();
                    break;
    
    
                case TURNING:
                    doTurningWithRadarRayCheck();
                    break;
    
                case UTURN:
                    doUTurnScan();
                    break;
    
                case BACKING_UP:
                    myMoveBack();
                    state = State.MOVE;
                    break;
                case CONVERGING:
                    if (commitForwardSteps > 0 && detectFront().getObjectType() == IFrontSensorResult.Types.NOTHING) {
                        myMove();
                        commitForwardSteps--;
                        break;
                    }
                    if (tryEngageFrontMainBot()) {
                        return;
                    }
                    if (currentTargetX != -1 && currentTargetY != -1) {
                        meetAtPoint(currentTargetX, currentTargetY, TARGET_PRECISION);
                    } else {
                        state = State.MOVE;
                    }
                    break;
                case IDLE:
                    break;
            }
        }
    
        // ==========================================================
        // MAIN MOVE RULE:
        // - If front is blocked => avoid (progressive) using this logic
        // - If front is safe => check RADAR_RADIUS (360°). If too close to anything, rotate bit-by-bit
        //   until the *current candidate direction* is not blocked by an obstacle ray, then move.
        // ==========================================================
        private void moveUsingFrontThenRadarRadius() {
            IFrontSensorResult front = detectFront();
    
            // If we just found a free heading previously, force 1 forward step (as long as front is clear)
            if (commitForwardSteps > 0 && front.getObjectType() == IFrontSensorResult.Types.NOTHING) {
                myMove();
                commitForwardSteps--;
                return;
            }
    
            // WALL => UTURN scan (360° search for a free direction)
            if (front.getObjectType() == IFrontSensorResult.Types.WALL) {
                afterTurnState = State.MOVE;
                enterUTurnMode();
                return;
            }
    
            // FRONT blocked (enemy bot / wreck / etc.)
            if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
                afterTurnState = State.MOVE;
                handleFrontBlockedNonWall();
                return;
            }
    
            // FRONT is clear => now RADAR RADIUS bubble (360°)
            if (radarHasAnythingWithinRadius(RADAR_RADIUS)) {
                afterTurnState = State.MOVE;
                // pick a side based on the closest obstacle around us (real radius), then start scanning
                Integer side = chooseSideFromClosestRadarWithinRadius(RADAR_RADIUS);
                if (side != null) avoidSide = side;
    
                // start with a small turn; TURNING will keep stepping until the chosen ray is no longer blocked
                targetAngle = normalize(myGetHeading() + avoidSide * AVOID_STEP);
                turnUsesRadarRayCheck = true;
                state = State.TURNING;
                return;
            }
    
            // Totally clear => move
            myMove();
            consecutiveBlocks = 0;
        }
    
        /* ==========================================================
            go to Point & reading messages
         */
        private void meetAtPoint(double x, double y, double precision) {
            if (x == -1 || y == -1) {
                state = State.MOVE;
                return;
            }
            double SEARCH_RADIUS = 350;
            double distance_to_point = Math.hypot(x - myX, y - myY);
    
            if (distance_to_point < SEARCH_RADIUS && currentTargetX != -1 && currentTargetY != -1 && nav_Lock) {
                double targetX = -1;
                double targetY = -1;
                double dist = 1e7;
    
                for(IRadarResult o : detectRadar()) {
                    if (o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                            o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                        double enemyAbsoluteX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
                        double enemyAbsoluteY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
    
                        // Only lock onto enemies that are reasonably close to where we expect them
                        double distanceFromExpected = Math.hypot(enemyAbsoluteX - currentTargetX, enemyAbsoluteY - currentTargetY);
                        if(distanceFromExpected < 300 && distanceFromExpected < dist) { // Within 300 units of expected
                            dist = distanceFromExpected;
                            targetX = enemyAbsoluteX;
                            targetY = enemyAbsoluteY;
                        }
                    }
                }
    
                if(targetX != -1 && targetY != -1) {
                    currentTargetX = targetX;
                    currentTargetY = targetY;
                    enemy_Lock = true;
                    nav_Lock = true;
                    enemy_wait_time = WAIT_ENEMY_TIME;
                    sendLogMessage(robotName + " >>> Enemy acquired at (x=" + (int) currentTargetX + ", y=" + (int) currentTargetY + ")");
                    stepsSinceEnemyUpdate = 0;
                    // Continue converging to updated position
                }
            }
            if (distance_to_point < precision) {
                if (returningHome) {
                    returningHome = false;
                    nav_Lock = false;
                    currentTargetX = -1;
                    currentTargetY = -1;
                    state = State.WAITING_FOR_SIGNAL; // or MOVE
                    return;
                }
                // we're there and no enemies
    //            abandonCurrentTarget(); // to be safe
                sendLogMessage(robotName + " >>> Arrived near target!");
                state = State.MOVE;
                return;
            }
    
            double angleToTarget = normalize(Math.atan2(y - myY, x - myX));
    
            // turn toward target first
            if (!isSameDirection(myGetHeading(), angleToTarget)) {
                targetAngle = angleToTarget;
                turnUsesRadarRayCheck = false;     // pure turning, not scanning
                afterTurnState = State.CONVERGING; // come back here after turn
                state = State.TURNING;
                return;
            }
    
            // aligned: if wall -> UTURN (then come back to converging)
            IFrontSensorResult front = detectFront();
            if (front.getObjectType() == IFrontSensorResult.Types.WALL) {
                afterTurnState = State.CONVERGING;
                enterUTurnMode();
                return;
            }
    
            // aligned: if blocked by something -> progressive avoid (then come back to converging)
            if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
                afterTurnState = State.CONVERGING;
                handleFrontBlockedNonWall();
                return;
            }
    
            // optional: radar bubble avoidance while converging too
            if (radarHasAnythingWithinRadius(RADAR_RADIUS)) {
                Integer side = chooseSideFromClosestRadarWithinRadius(RADAR_RADIUS);
                if (side != null) avoidSide = side;
    
                targetAngle = normalize(myGetHeading() + avoidSide * AVOID_STEP);
                turnUsesRadarRayCheck = true;
                afterTurnState = State.CONVERGING;
                state = State.TURNING;
                return;
            }
    
            // clear -> move
            myMove();
            consecutiveBlocks = 0;
        }
    
        private boolean lastChanceEnemyCheck() {
            if(tryEngageFrontMainBot()){
                return true;
            }
            IRadarResult close = findClosestEnemyOnRadar();
            if(close != null){
                double ex = myX + close.getObjectDistance()
                        * Math.cos(close.getObjectDirection());
                double ey = myY + close.getObjectDistance()
                        * Math.sin(close.getObjectDirection());
    
                lockTarget(ex, ey, getEnemyTypeRadar(close));
                return shootAtCurrentTarget();
            }
            return false;
        }
        private IRadarResult findClosestEnemyOnRadar() {
            IRadarResult closest = null;
            double bestDist = Double.POSITIVE_INFINITY;
    
            for (IRadarResult o : detectRadar()) {
                if (o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                        o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                    if (o.getObjectDistance() < bestDist) {
                        bestDist = o.getObjectDistance();
                        closest = o;
                    }
                }
            }
            return closest;
        }
        private boolean tryEngageFrontMainBot() {
            IFrontSensorResult f = detectFront();
    
            if (f.getObjectType() != IFrontSensorResult.Types.OpponentMainBot) {
                return false;
            }
    
            if (enemy_Lock) {
                return shootAtCurrentTarget();
            }
    
            IRadarResult frontEnemy =
                    findEnemyClosestToHeading(myGetHeading(), FRONT_ATTACK_CONE);
    
            if (frontEnemy != null) {
                double ex = myX + frontEnemy.getObjectDistance()
                        * Math.cos(frontEnemy.getObjectDirection());
                double ey = myY + frontEnemy.getObjectDistance()
                        * Math.sin(frontEnemy.getObjectDirection());
                lockTarget(ex, ey, getEnemyTypeRadar(frontEnemy));
                return shootAtCurrentTarget();
            }
    
            return false;
        }

        private void maybeUpgradeToMainBot() {
            if (currentTargetType != TargetType.ENEMY_SECONDARY) return;

            IRadarResult enemy = findVisibleMainBot(); // your helper
            if (enemy == null) return;

            double enemyX = myX + enemy.getObjectDistance() * Math.cos(enemy.getObjectDirection());
            double enemyY = myY + enemy.getObjectDistance() * Math.sin(enemy.getObjectDirection());

            lockTarget(enemyX, enemyY, TargetType.ENEMY_MAIN);
        }

        private IRadarResult findVisibleMainBot() {
            IRadarResult best = null;
            double bestDist = Double.POSITIVE_INFINITY;
    
            for (IRadarResult o : detectRadar()) {
                if (o.getObjectType() != IRadarResult.Types.OpponentMainBot) continue;
                if (o.getObjectDistance() < bestDist) {
                    bestDist = o.getObjectDistance();
                    best = o;
                }
            }
            return best;
        }
    
    
        private TargetType getEnemyTypeRadar(IRadarResult enemy) {
            if (enemy == null) return TargetType.NONE;
            switch (enemy.getObjectType()) {
                case OpponentMainBot:      return TargetType.ENEMY_MAIN;
                case OpponentSecondaryBot: return TargetType.ENEMY_SECONDARY;
                default:                  return TargetType.NONE;
            }
        }
    
        private TargetType getEnemyTypeFront(IFrontSensorResult front) {
            if (front == null) return TargetType.NONE;
            switch (front.getObjectType()) {
                case OpponentMainBot:      return TargetType.ENEMY_MAIN;
                case OpponentSecondaryBot: return TargetType.ENEMY_SECONDARY;
                default:                  return TargetType.NONE;
            }
        }
    
    
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
            boolean busy = enemy_Lock || (currentTargetX != -1 && currentTargetY != -1) || nav_Lock;
            for (String msg : messages) {
                if (msg.startsWith("ENEMY_LOCATION|")) {
                    try {
                        if (busy){
                            sendLogMessage("" + robotName + " BUSY, ignoring ENEMY message");
                            continue;
                        }
                        String[] parts = msg.split("\\|");
                        if (parts.length == 6) {
                            String spotter = parts[1];
                            if (robotName.equals(spotter)) continue;
    
                            double enemyX = Double.parseDouble(parts[4]);
                            double enemyY = Double.parseDouble(parts[5]);
                            stepsSinceEnemyUpdate = 0;
                            nav_Lock = true;
                            sendLogMessage(robotName + " ENEMY from " + spotter+
                                    " (x=" + (int) enemyX + ", y=" + (int) enemyY + ")");
    //                        applyFormationOffset(spotter, enemyX, enemyY);
                            applyFormationOffsetAngle(enemyX,enemyY);
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
                        if (busy){
                            sendLogMessage("" + robotName + " BUSY, ignoring ENEMY message");
                            continue;
                        }
                        String[] parts = msg.split("\\|");
                        if (parts.length == 6) {
                            String spotter = parts[1];
                            double enemyX = Double.parseDouble(parts[4]);
                            double enemyY = Double.parseDouble(parts[5]);
                            stepsSinceEnemyUpdate = 0;
                            nav_Lock = true;
                            sendLogMessage(robotName + " ENEMY from " + spotter+
                                    " (x=" + (int) enemyX + ", y=" + (int) enemyY + ")");
    //                        applyFormationOffset(spotter, enemyX, enemyY);
                            applyFormationOffsetAngle(enemyX,enemyY);
                        }
                    } catch (Exception ignored) {}
                }
            }
        }
        private void applyFormationOffset(String spotter, double targetX, double targetY) {
            int spotterPosition = getRolePosition(spotter);
            int myPosition = getRolePosition(robotName);
    
            int relativeOffset = (myPosition - spotterPosition);
    
            currentTargetX = targetX + (relativeOffset * FLANK_OFFSET_X);
            currentTargetY = targetY;
        }
        private void applyFormationOffsetAngle(double targetX, double targetY) {
            int rel = getRolePosition(robotName); // WARIO=-1, MARIO=0, LUIGI=+1
    
            // axis is "me -> target"
            double base = Math.atan2(targetY - myY, targetX - myX);
    
            // we want points a bit *around* the target, not inside it
            // choose "behind target" relative to me, so we don't collide head-on
            double behind = base + Math.PI;
    
            double angleOffset = (rel == 0) ? 0 : (rel > 0 ? +FLANK_ANGLE : -FLANK_ANGLE);
            double a = behind + angleOffset;
    
            currentTargetX = targetX + FLANK_RADIUS * Math.cos(a);
            currentTargetY = targetY + FLANK_RADIUS * Math.sin(a);
        }
    
        private int getRolePosition(String name) {
            switch (name) {
                case "WARIO": return -1;
                case "MARIO": return 0;
                case "LUIGI": return 1;
                default:      return 0;
            }
        }
        private void goBackToDefaultPosition() {
            if (defaultX != -1 && defaultY != -1) {
                applyFormationOffsetAngle(defaultX, defaultY); // sets currentTargetX/Y
                nav_Lock = true;
                enemy_Lock = false;
                enemy_wait_time = -1;  // important: don't trigger TTL logic
                stepsSinceEnemyUpdate = 0; // optional: avoid instant cooldown weirdness
                state = State.CONVERGING;
            }
        }
        // FRONT blocked by something that isn't a wall
        private void handleFrontBlockedNonWall() {
            consecutiveBlocks++;
    
            // try to pick a side away from the closest radar obstacle (within radius), else alternate
            Integer side = chooseSideFromClosestRadarWithinRadius(RADAR_RADIUS);
            if (side != null) avoidSide = side;
            else if (consecutiveBlocks % 2 == 0) avoidSide = -avoidSide;
    
            // progressive target: 30, 60, 90, 120...
            double turnAmount = progressiveTurn(consecutiveBlocks);
            targetAngle = normalize(myGetHeading() + avoidSide * turnAmount);
    
            // safety escape
            if (consecutiveBlocks >= BLOCK_ESCAPE_TRIGGER) {
                escapeBackSteps = ESCAPE_BACK_STEPS;
                avoidSide = -avoidSide;
                // after backing, force a 90° change (3 * 30°)
                targetAngle = normalize(myGetHeading() + avoidSide * (3 * AVOID_STEP));
                consecutiveBlocks = 0;
            }
    
            turnUsesRadarRayCheck = true;
            state = State.TURNING;
        }
    
        // TURNING behavior:
        // - turn toward targetAngle
        // - when aligned, ask: "is this direction still blocked by a radar obstacle ray?"
        //   if yes -> rotate another 30° and try again
        //   if no  -> commit 1 forward step
        private void doTurningWithRadarRayCheck() {
            if (!isSameDirection(myGetHeading(), targetAngle)) {
                stepTurn(getTurnDirection(myGetHeading(), targetAngle));
                return;
            }
    
            // aligned
            if (turnUsesRadarRayCheck) {
                if (radarBlocksHeading(targetAngle, RADAR_RADIUS)) {
                    targetAngle = normalize(targetAngle + avoidSide * AVOID_STEP);
                    return;
                } else {
                    commitForwardSteps = 1;
                    turnUsesRadarRayCheck = false;
                    state = afterTurnState;  // ✅ instead of MOVE
                    return;
                }
            }
    
            state = afterTurnState;          // ✅ instead of MOVE
        }
    
    
        // ==========================================================
        // UTURN MODE (wall): start facing 180° away, then scan 360° in 30° steps.
        // if front becomes clear => move + exit
        // ==========================================================
        private void enterUTurnMode() {
            escapeBackSteps = Math.max(escapeBackSteps, 1);
    
            // alternate scan direction to avoid corner loops
            uTurnSide = -uTurnSide;
    
            // base direction: directly away from the wall (front saw wall)
            uTurnStartHeading = normalize(myGetHeading() + Math.PI);
    
            uTurnChecksLeft = (int) Math.ceil((2 * Math.PI) / AVOID_STEP);
            targetAngle = uTurnStartHeading;
    
            consecutiveBlocks = 0;
            turnUsesRadarRayCheck = false;
            state = State.UTURN;
        }
    
        private void doUTurnScan() {
            if (!isSameDirection(myGetHeading(), targetAngle)) {
                stepTurn(getTurnDirection(myGetHeading(), targetAngle));
                return;
            }
    
            if (detectFront().getObjectType() == IFrontSensorResult.Types.NOTHING) {
                // found a free direction
                commitForwardSteps = 1;
                state = afterTurnState;
                return;
            }
    
            uTurnChecksLeft--;
            if (uTurnChecksLeft <= 0) {
                // full scan failed: back up more and try moving logic again
                escapeBackSteps = Math.max(escapeBackSteps, 2);
                state = afterTurnState;
                return;
            }
    
            int totalSteps = (int) Math.ceil((2 * Math.PI) / AVOID_STEP);
            int stepIndex = totalSteps - uTurnChecksLeft + 1;
            targetAngle = normalize(uTurnStartHeading + uTurnSide * stepIndex * AVOID_STEP);
        }
    
        // ==========================================================
        // RADAR HELPERS (REAL RADIUS + RAY-BLOCK CHECK)
        // ==========================================================
        private boolean radarHasAnythingWithinRadius(double radius) {
            for (IRadarResult o : detectRadar()) {
                if (!isWantedRadarType(o.getObjectType())) continue;
                if (o.getObjectDistance() < radius) return true;
            }
            return false;
        }
    
        private boolean radarBlocksHeading(double candidateHeading, double radius) {
            for (IRadarResult o : detectRadar()) {
                if (!isWantedRadarType(o.getObjectType())) continue;
                if (o.getObjectDistance() >= radius) continue;
    
                // obstacle lies on this candidate ray => blocked
                if (isSameDirectionRadar(o.getObjectDirection(), candidateHeading)) return true;
            }
            return false;
        }
    
        // pick side away from the closest obstacle inside radius (360°)
        // returns -1 (turn left) or +1 (turn right) or null if none
        private Integer chooseSideFromClosestRadarWithinRadius(double radius) {
            IRadarResult closest = null;
            double best = Double.POSITIVE_INFINITY;
    
            for (IRadarResult o : detectRadar()) {
                if (!isWantedRadarType(o.getObjectType())) continue;
                double d = o.getObjectDistance();
                if (d >= radius) continue;
    
                if (d < best) {
                    best = d;
                    closest = o;
                }
            }
    
            if (closest == null) return null;
    
            double rel = normalize(closest.getObjectDirection() - myGetHeading());
            // object on right (0..pi) => turn left (-1). object on left => turn right (+1)
            return (rel > 0 && rel < Math.PI) ? -1 : +1;
        }
    
        private boolean isWantedRadarType(IRadarResult.Types t) {
            return (t == IRadarResult.Types.Wreck)
                    || (t == IRadarResult.Types.TeamMainBot)
                    || (t == IRadarResult.Types.OpponentMainBot)
                    || (t == IRadarResult.Types.TeamSecondaryBot)
                    || (t == IRadarResult.Types.OpponentSecondaryBot);
        }
    
        private boolean isSameDirectionRadar(double dir1, double dir2) {
            return Math.abs(normalize(dir1) - normalize(dir2)) < RADAR_DIR_EPS;
        }
    
        // progressive turn: 30, 60, 90, 120...
        private double progressiveTurn(int blocks) {
            double turn = blocks * AVOID_STEP;
            double maxTurn = 2 * Math.PI - AVOID_STEP; // up to 330°
            if (turn > maxTurn) turn = maxTurn;
            return turn;
        }
    
        private IRadarResult findEnemyClosestToHeading(double heading, double maxAngle) {
            IRadarResult best = null;
            double bestAbs = Double.POSITIVE_INFINITY;
    
            for (IRadarResult o : detectRadar()) {
                if (o.getObjectType() != IRadarResult.Types.OpponentMainBot) continue;
    
                double dAng = Math.abs(normalize(o.getObjectDirection() - heading));
                if (dAng < maxAngle && dAng < bestAbs) {
                    bestAbs = dAng;
                    best = o;
                }
            }
            return best;
        }
    
    
        // ATTACKING FUNCTIONS
            private boolean updateTargetFromRadarIfVisible(){
                double enemyX = -1;
                double enemyY = -1;
                double dist = Double.POSITIVE_INFINITY;
                for (IRadarResult o : detectRadar()) {
                    if (o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                            o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                        double enemyAbsoluteX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
                        double enemyAbsoluteY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
                        if(Math.hypot(enemyAbsoluteX - currentTargetX, enemyAbsoluteY - currentTargetY) < dist) {
                            dist = Math.hypot(enemyAbsoluteX - currentTargetX, enemyAbsoluteY - currentTargetY);
                            enemyX = enemyAbsoluteX;
                            enemyY = enemyAbsoluteY;
                        }
                    }
                }
                if(enemyX != -1 && enemyY != -1) {
                    stepsSinceEnemyUpdate = 0;
                    currentTargetX = enemyX;
                    currentTargetY = enemyY;
                    enemy_Lock = true;
                    nav_Lock = true;
                    enemy_wait_time = WAIT_ENEMY_TIME;
                    return true;
                }
                return false;
            }
        private boolean shootAtCurrentTarget() {
            if (currentTargetX == -1 || currentTargetY == -1) return false;

            // incase there is a main
            maybeUpgradeToMainBot();

            double angleToTarget = normalize(Math.atan2(currentTargetY - myY, currentTargetX - myX));

            sendLogMessage("shooting at (x=" + (int) currentTargetX + ", y=" + (int) currentTargetY + ")");
            if(teammateBlocksShot(angleToTarget)) {
                avoidSide = (avoidSide == 0) ? 1 : avoidSide;
                targetAngle = normalize(angleToTarget + avoidSide * (AVOID_STEP / 3));
                afterTurnState = State.CONVERGING;
                turnUsesRadarRayCheck = false;
                state = State.TURNING;
                return false;
            }
            // aligned: shoot
            fire(angleToTarget);
            return true;
        }
        // ==========================================================
        // MOVEMENT + ODOMETRY
        // ==========================================================
        private void myMove() {
            isMoving = true;
            lastMoveWasBack = false;
            move();
        }
    
        private void myMoveBack() {
            isMoving = true;
            lastMoveWasBack = true;
            moveBack();
        }
    
        private void updateOdometry() {
            if (!isMoving) return;
    
            if (detectFront().getObjectType() != IFrontSensorResult.Types.WALL) {
                double s = Parameters.teamBMainBotSpeed;
                if (lastMoveWasBack) s = -s;
    
                myX += s * Math.cos(myGetHeading());
                myY += s * Math.sin(myGetHeading());
    //            sendLogMessage(robotName + " (x=" + (int) myX + ", y=" + (int) myY + ")");
            }
    
            isMoving = false;
        }
    
        // ==========================================================
        // TURN HELPERS
        // ==========================================================
        private Parameters.Direction getTurnDirection(double current, double target) {
            double diff = normalize(target - current);
            return (diff <= Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
        }
    
        private boolean isSameDirection(double dir1, double dir2) {
            return Math.abs(normalize(dir1) - normalize(dir2)) < ANGLE_PRECISION;
        }
    
        private double myGetHeading() {
            double h = getHeading();
            while (h < 0) h += 2 * Math.PI;
            while (h >= 2 * Math.PI) h -= 2 * Math.PI;
            return h;
        }
    
        private double normalize(double dir) {
            double res = dir;
            while (res < 0) res += 2 * Math.PI;
            while (res >= 2 * Math.PI) res -= 2 * Math.PI;
            return res;
        }
        private boolean inRange(double a, double b, double range) {
            return Math.abs(a - b) <= range;
        }
    
        private boolean teammateBlocksShot(double angle) {
            for (IRadarResult o : detectRadar()) {
                if (o.getObjectType() == IRadarResult.Types.TeamMainBot
                        || o.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
    
                    if (isSameDirectionRadar(o.getObjectDirection(), angle)) {
                        return true;
                    }
                }
            }
            return false;
        }
    }
