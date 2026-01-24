    package algorithms;

    import characteristics.IFrontSensorResult;
    import characteristics.IRadarResult;
    import characteristics.Parameters;
    import robotsimulator.Brain;

    import java.util.ArrayList;

    public class RobotB extends Brain {

        private enum Role { WARIO, MARIO, LUIGI, UNDEFINED }
        private enum State { IDLE, MOVING, ATTACKING, CONVERGING, STOPPED, EVADING }

        // --- IDENTITY & STATE ---
        private Role role = Role.UNDEFINED;
        private String robotName = "Unknown";
        private State state = State.IDLE;

        // --- POSITION & MOVEMENT ---
        private double myX, myY;
        private double targetX = -1, targetY = -1;
        private double targetAngle;
        private double targetPrecision = 100;
        private int moveSign = 0; // +1 forward, -1 backward, 0 none

        // --- TIMING & COUNTERS ---
        private int evadeSteps = 0;
        private int consecutiveBlocked = 0;
        private int stepsSinceEnemyUpdate = 0;
        private int noEnemySignalCooldown = 0;

        // --- CONSTANTS ---
        private static final double ANGLE_PRECISION = 0.1;
        private static final double TURN_INCREMENT = Math.PI / 6; // 30°
        private static final int ENEMY_INFO_EXPIRY = 20;
        private static final int STOPPED_TIME = 300;
        private static final int EVADE_DURATION = 15;

        // Detection ranges
        private static final int ENEMY_RANGE = 400;
        private static final int WRECK_RANGE = 100;
        private static final int TEAMMATE_RANGE = 120;
        private static final int SCOUT_RANGE = 80;

        // Tactical positioning
        private static final double FLANK_OFFSET_X = 150;

        // Map bounds
        private double northBound = -1, westBound = -1, eastBound = -1, southBound = -1;
        private double DEFAULTX = 0, DEFAULTY = 0;
        private static final double BORDER_MARGIN = 100.0;

        // --- SHARED ENEMY TRACKING ---
        private double sharedEnemyX = -1;
        private double sharedEnemyY = -1;

        // --- WRECK MEMORY ---
        private static class WreckInfo {
            double x, y;
            WreckInfo(double x, double y) { this.x = x; this.y = y; }
        }
        private final ArrayList<WreckInfo> knownWrecks = new ArrayList<>();
        private static final double WRECK_TOLERANCE = 25.0;

        @Override
        public void activate() {
            identifyRole();
            DEFAULTX = Parameters.teamBMainBot2InitX - 300;
            DEFAULTY = Parameters.teamBMainBot2InitY;

            state = State.STOPPED;
            targetAngle = myGetHeading();
            noEnemySignalCooldown = STOPPED_TIME;
            knownWrecks.clear();
        }

        public void identifyRole() {
            // Determine identity by radar
            boolean seesNorth = false, seesSouth = false;
            for (IRadarResult o : detectRadar()) {
                if (o.getObjectType() == IRadarResult.Types.TeamMainBot) {
                    if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) seesNorth = true;
                    if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH)) seesSouth = true;
                }
            }

            int whoAmI = seesNorth && !seesSouth ? 3 : (seesSouth && !seesNorth ? 1 : 2);

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
            }
        }

        @Override
        public void step() {
            updateOdometry();
            rememberWrecks();
            readTeammateMessages();

            // Update enemy info expiry
            if (++stepsSinceEnemyUpdate > ENEMY_INFO_EXPIRY) {
                sharedEnemyX = sharedEnemyY = -1;
                if (state == State.CONVERGING) state = State.MOVING;
            }

            noEnemySignalCooldown = Math.max(0, noEnemySignalCooldown - 1);

            // Priority: attacking overrides everything
            if (enemyCheck() && state != State.ATTACKING) {
                state = State.ATTACKING;
            }

            switch (state) {
                case STOPPED:
                    if (noEnemySignalCooldown <= 0) state = State.IDLE;
                    break;

                case IDLE:
                    if (sharedEnemyX != -1 && sharedEnemyY != -1) {
                        setTarget(sharedEnemyX, sharedEnemyY, 100);
                        state = State.CONVERGING;
                    } else {
                        setTarget(DEFAULTX, DEFAULTY, 100);
                        state = State.MOVING;
                    }
                    break;

                case MOVING:
                case CONVERGING:
                    if (!moveToTarget()) {
                        startEvasion();
                    }
                    break;

                case ATTACKING:
                    shootEnemy();
                    if (!enemyCheck()) {
                        state = sharedEnemyX != -1 ? State.CONVERGING : State.MOVING;
                    }
                    break;

                case EVADING:
                    if (evadeSteps > 0) {
                        performEvasion();
                        evadeSteps--;
                    } else {
                        // Resume previous mission
                        state = (sharedEnemyX != -1) ? State.CONVERGING : State.MOVING;
                        consecutiveBlocked = 0;
                    }
                    break;
            }
        }

        // ============================================================
        // UNIFIED MOVEMENT SYSTEM
        // ============================================================

        private void setTarget(double x, double y, double precision) {
            targetX = x;
            targetY = y;
            targetPrecision = precision;
        }

        /**
         * Move towards current target. Returns false if blocked.
         */
        private boolean moveToTarget() {
            if (targetX == -1 || targetY == -1) return true;

            double distance = Math.hypot(targetX - myX, targetY - myY);

            // Reached target
            if (distance < targetPrecision) {
                if (targetX == DEFAULTX && targetY == DEFAULTY) {
                    state = State.STOPPED;
                    noEnemySignalCooldown = STOPPED_TIME;
                } else {
                    state = State.MOVING;
                }
                return true;
            }

            // Calculate angle to target
            double angleToTarget = Math.atan2(targetY - myY, targetX - myX);

            // Turn if needed
            if (!isSameDirection(myGetHeading(), angleToTarget)) {
                turnTowards(angleToTarget);
                return true;
            }

            // Move if clear, otherwise signal blockage
            if (!isBlocked()) {
                myMove();
                consecutiveBlocked = 0;
                return true;
            }

            consecutiveBlocked++;
            return false;
        }

        private void turnTowards(double targetAngle) {
            double diff = normalize(targetAngle - myGetHeading());
            Parameters.Direction dir = (diff < Math.PI) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT;
            stepTurn(dir);
        }

        // ============================================================
        // EVASION SYSTEM
        // ============================================================

        private void startEvasion() {
            state = State.EVADING;
            evadeSteps = EVADE_DURATION;
            targetAngle = calculateEvasionAngle();
            sendLogMessage(robotName + " >>> Starting evasion");
        }

        private void performEvasion() {
            // First half: back up
            if (evadeSteps > EVADE_DURATION / 2) {
                myMoveBack();
            }
            // Second half: turn and advance
            else {
                if (!isSameDirection(myGetHeading(), targetAngle)) {
                    turnTowards(targetAngle);
                } else {
                    myMove();
                }
            }
        }

        private double calculateEvasionAngle() {
            double heading = myGetHeading();

            // Wall handling
            if (detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
                if (nearKnownBorder()) return angleTowardKnownCenter();
                return normalize(heading + (consecutiveBlocked % 2 == 0 ? Math.PI / 2 : -Math.PI / 2));
            }

            // Scan obstacles
            int leftScore = 0, rightScore = 0;
            for (IRadarResult o : detectRadar()) {
                if (isBehind(o.getObjectDirection()) || o.getObjectDistance() > 250) continue;

                double rel = normalize(o.getObjectDirection() - heading);
                boolean isLeft = (rel > Math.PI);
                int weight = (o.getObjectType() == IRadarResult.Types.Wreck) ? 2 : 1;

                if (isLeft) leftScore += weight;
                else rightScore += weight;
            }

            // Choose less crowded side
            if (leftScore < rightScore) return normalize(heading - TURN_INCREMENT);
            if (rightScore < leftScore) return normalize(heading + TURN_INCREMENT);
            return normalize(heading + (consecutiveBlocked % 2 == 0 ? TURN_INCREMENT : -TURN_INCREMENT));
        }

        // ============================================================
        // ODOMETRY & BASIC MOVEMENT
        // ============================================================

        private void updateOdometry() {
            if (moveSign != 0) {
                boolean blocked = isBlockedByWreck() || isBlockedByTeammate() || isBlockedByOpponent()
                        || (moveSign > 0 && detectWall());

                if (!blocked) {
                    double step = Parameters.teamAMainBotSpeed * moveSign;
                    myX += step * Math.cos(myGetHeading());
                    myY += step * Math.sin(myGetHeading());
                }
                moveSign = 0;
            }
        }

        private void myMove() {
            moveSign = 1;
            move();
        }

        private void myMoveBack() {
            moveSign = -1;
            moveBack();
        }

        // ============================================================
        // DETECTION METHODS
        // ============================================================

        private boolean isBlocked() {
            return detectWall() || isBlockedByWreck() || isBlockedByTeammate() || isBlockedByOpponent();
        }

        private boolean detectWall() {
            return detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
        }

        private boolean isBlockedByWreck() {
            return detectInFront(IRadarResult.Types.Wreck, WRECK_RANGE);
        }

        private boolean isBlockedByTeammate() {
            return detectInFront(IRadarResult.Types.TeamMainBot, TEAMMATE_RANGE)
                    || detectInFront(IRadarResult.Types.TeamSecondaryBot, SCOUT_RANGE)
                    || detectInFront(IRadarResult.Types.OpponentSecondaryBot, SCOUT_RANGE);
        }

        private boolean isBlockedByOpponent() {
            return detectInFront(IRadarResult.Types.OpponentMainBot, ENEMY_RANGE)
                    || detectInFront(IRadarResult.Types.OpponentSecondaryBot, ENEMY_RANGE);
        }

        private boolean detectInFront(IRadarResult.Types type, int range) {
            for (IRadarResult o : detectRadar()) {
                if (o.getObjectType() == type && isInFront(o.getObjectDirection())
                        && !isBehind(o.getObjectDirection()) && o.getObjectDistance() < range) {
                    return true;
                }
            }
            return false;
        }

        private boolean isInFront(double objDir) {
            double relativeAngle = normalize(objDir - myGetHeading());
            return relativeAngle < Math.PI / 4 || relativeAngle > (2 * Math.PI - Math.PI / 4);
        }

        private boolean isBehind(double objDir) {
            double relativeAngle = normalize(objDir - myGetHeading());
            return relativeAngle > (3 * Math.PI / 4) && relativeAngle < (5 * Math.PI / 4);
        }

        // ============================================================
        // COMBAT METHODS
        // ============================================================

        private void shootEnemy() {
            IRadarResult target = null;
            boolean foundShooter = false;

            for (IRadarResult o : detectRadar()) {
                if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot
                        || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot)
                        && o.getObjectDistance() <= ENEMY_RANGE) {

                    if (isBlockedByWreckLine(o.getObjectDirection(), o.getObjectDistance())) continue;

                    boolean isShooter = (o.getObjectType() == IRadarResult.Types.OpponentMainBot);
                    if (target == null || (isShooter && !foundShooter) || o.getObjectDistance() < target.getObjectDistance()) {
                        target = o;
                        foundShooter = isShooter;
                    }
                }
            }

            if (target != null) {
                fire(target.getObjectDirection());
                broadcastEnemyPosition(target);
            }
        }

        private boolean enemyCheck() {
            for (IRadarResult o : detectRadar()) {
                if ((o.getObjectType() == IRadarResult.Types.OpponentMainBot
                        || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot)
                        && o.getObjectDistance() <= ENEMY_RANGE
                        && !isBlockedByWreckLine(o.getObjectDirection(), o.getObjectDistance())) {
                    return true;
                }
            }
            return false;
        }

        private boolean isBlockedByWreckLine(double enemyDir, double enemyDist) {
            double botRadius = Parameters.teamAMainBotRadius;
            for (IRadarResult wreck : detectRadar()) {
                if (wreck.getObjectType() == IRadarResult.Types.Wreck && wreck.getObjectDistance() < enemyDist) {
                    double angularWidth = Math.atan(botRadius / wreck.getObjectDistance());
                    double angleDiff = Math.abs(normalize(wreck.getObjectDirection() - enemyDir));
                    if (angleDiff > Math.PI) angleDiff = 2 * Math.PI - angleDiff;
                    if (angleDiff <= angularWidth) return true;
                }
            }
            return false;
        }

        // ============================================================
        // COMMUNICATION
        // ============================================================

        private void broadcastEnemyPosition(IRadarResult enemy) {
            double enemyAbsoluteX = myX + enemy.getObjectDistance() * Math.cos(enemy.getObjectDirection());
            double enemyAbsoluteY = myY + enemy.getObjectDistance() * Math.sin(enemy.getObjectDirection());
            broadcast("ENEMY_LOCATION|" + robotName + "|" + (int)myX + "|" + (int)myY
                    + "|" + (int)enemyAbsoluteX + "|" + (int)enemyAbsoluteY);
        }

        private void readTeammateMessages() {
            for (String msg : fetchAllMessages()) {
                if (msg.startsWith("ENEMY_LOCATION|") || msg.startsWith("SCOUT_ENEMY_LOCATION")) {
                    parseEnemyMessage(msg);
                } else if (msg.startsWith("BORDER")) {
                    parseBorderMessage(msg);
                }
            }
        }

        private void parseEnemyMessage(String msg) {
            try {
                String[] parts = msg.split("\\|");
                if (parts.length == 6 && !robotName.equals(parts[1])) {
                    double enemyX = Double.parseDouble(parts[4]);
                    double enemyY = Double.parseDouble(parts[5]);
                    stepsSinceEnemyUpdate = 0;
                    applyFormationOffset(parts[1], enemyX, enemyY);
                }
            } catch (Exception ignored) {}
        }

        private void parseBorderMessage(String msg) {
            try {
                String[] parts = msg.split("\\|");
                if (parts.length == 3) {
                    int pos = Integer.parseInt(parts[2]);
                    switch (parts[1]) {
                        case "NORTH": northBound = pos; break;
                        case "SOUTH": southBound = pos; break;
                        case "WEST": westBound = pos; break;
                        case "EAST": eastBound = pos; break;
                    }
                }
            } catch (Exception ignored) {}
        }

        private void applyFormationOffset(String spotter, double targetX, double targetY) {
            int relativeOffset = getRolePosition(robotName) - getRolePosition(spotter);
            sharedEnemyX = targetX + (relativeOffset * FLANK_OFFSET_X);
            sharedEnemyY = targetY;
        }

        private int getRolePosition(String name) {
            switch (name) {
                case "WARIO": return -1;
                case "LUIGI": return 1;
                default: return 0;
            }
        }

        // ============================================================
        // WRECK & BORDER HELPERS
        // ============================================================

        private void rememberWrecks() {
            for (IRadarResult o : detectRadar()) {
                if (o.getObjectType() != IRadarResult.Types.Wreck) continue;

                double wx = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
                double wy = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());

                boolean alreadyKnown = knownWrecks.stream()
                        .anyMatch(w -> Math.hypot(w.x - wx, w.y - wy) <= WRECK_TOLERANCE);

                if (!alreadyKnown) knownWrecks.add(new WreckInfo(wx, wy));
            }
        }

        private boolean nearKnownBorder() {
            double best = Double.POSITIVE_INFINITY;
            if (northBound != -1) best = Math.min(best, Math.abs(northBound - myY));
            if (southBound != -1) best = Math.min(best, Math.abs(myY - southBound));
            if (eastBound != -1) best = Math.min(best, Math.abs(eastBound - myX));
            if (westBound != -1) best = Math.min(best, Math.abs(myX - westBound));
            return best <= BORDER_MARGIN;
        }

        private double angleTowardKnownCenter() {
            double cx = (westBound != -1 && eastBound != -1) ? (westBound + eastBound) / 2.0 : DEFAULTX;
            double cy = (southBound != -1 && northBound != -1) ? (southBound + northBound) / 2.0 : DEFAULTY;
            return Math.atan2(cy - myY, cx - myX);
        }

        // ============================================================
        // UTILITY METHODS
        // ============================================================

        private boolean isSameDirection(double dir1, double dir2) {
            return Math.abs(normalize(dir1) - normalize(dir2)) < ANGLE_PRECISION;
        }

        private double myGetHeading() {
            double result = getHeading();
            while (result < 0) result += 2 * Math.PI;
            while (result >= 2 * Math.PI) result -= 2 * Math.PI;
            return result;
        }

        private double normalize(double dir) {
            double res = dir;
            while (res < 0) res += 2 * Math.PI;
            while (res >= 2 * Math.PI) res -= 2 * Math.PI;
            return res;
        }
    }