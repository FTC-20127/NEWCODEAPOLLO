package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 *
 * FIELD AXES (looking from the Audience wall toward the Scoring/Rear wall):
 *   Origin (0, 0) = center of the 144x144 in field (center of the HIVE frame)
 *   +X = RIGHT (toward Blue Alliance wall)   |  -X = LEFT (toward Red Alliance wall)
 *   +Y = AWAY from Audience (Scoring wall)   |  -Y = TOWARD Audience wall
 *   Heading: 0 deg = facing Blue (+X), +90 deg = facing Scoring (+Y),
 *            180 deg = facing Red (-X), -90 deg = facing Audience (-Y). (+ = CCW / LEFT)
 */
public class PinpointFieldTracker {

    public enum Alliance { RED, BLUE }

    // ---------- 1. PINPOINT SETTINGS (verify signs in Step 2) ----------
    public static String PINPOINT_NAME = "pinpoint";

    // xOffset (forward pod): + = LEFT of robot center, - = RIGHT of robot center
    // yOffset (strafe pod):  + = FORWARD of robot center, - = BACK of robot center
    public static double FORWARD_POD_OFFSET_MM = 90.267;
    public static double STRAFE_POD_OFFSET_MM  = 25.94945;

    public static GoBildaPinpointDriver.EncoderDirection X_POD_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection Y_POD_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // ---------- 2. TURRET PIVOT OFFSET FROM ROBOT CENTER (inches) ----------
    public static double TURRET_OFFSET_FORWARD_IN = 0.0;
    public static double TURRET_OFFSET_LEFT_IN    = 0.0;

    // ---------- 3. BIOBUZZ HIVE CELL CENTERS (inches from field center (0, 0)) ----------
    public static double RED_AUDIENCE_X  = -12.0;
    public static double RED_AUDIENCE_Y  = -14.0;
    public static double RED_SCORING_X   = -12.0;
    public static double RED_SCORING_Y   = +14.0;

    public static double BLUE_AUDIENCE_X = +12.0;
    public static double BLUE_AUDIENCE_Y = -14.0;
    public static double BLUE_SCORING_X  = +12.0;
    public static double BLUE_SCORING_Y  = +14.0;

    // ---------- 4. MATCH STARTING POSES (update to match where you place the robot) ----------
    // RED: Tile A6 — back on Red wall, left side on Scoring wall, facing Blue (+X)
    public static double RED_START_X_IN        = -63.0;
    public static double RED_START_Y_IN        = +63.0;
    public static double RED_START_HEADING_DEG = 0.0;

    // BLUE: Tile F1 — back on Blue wall, left side on Audience wall, facing Red (-X)
    public static double BLUE_START_X_IN        = +63.0;
    public static double BLUE_START_Y_IN        = -63.0;
    public static double BLUE_START_HEADING_DEG = 180.0;
    /** Constant aim tweak in degrees (+ aims more LEFT, - aims more RIGHT). */
    public static double SHOT_OFFSET_DEG = 0.0;

    // ---------- STATE ----------
    private final GoBildaPinpointDriver pinpoint;
    private final String configProblem;

    private Alliance alliance = Alliance.BLUE;
    private boolean audienceCellUp = false; // Match start: Red = Audience up, Blue = Scoring up

    private double xIn = 0.0;
    private double yIn = 0.0;
    private double headingDeg = 0.0;
    private double distanceToCellIn = Double.NaN;
    private double targetTurretAngleDeg = 0.0;
    private boolean ready = false;

    public PinpointFieldTracker(HardwareMap hardwareMap) {
        GoBildaPinpointDriver driver = null;
        String err = null;
        try {
            driver = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        } catch (RuntimeException e) {
            err = "no \"" + PINPOINT_NAME + "\" in robot config";
        }
        pinpoint = driver;
        configProblem = err;
    }

    /** Call in init() while robot is completely still. */
    public void init(Alliance startAlliance) {
        setAlliance(startAlliance);
        if (pinpoint == null) return;

        pinpoint.setOffsets(FORWARD_POD_OFFSET_MM, STRAFE_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_POD_DIR, Y_POD_DIR);
        pinpoint.resetPosAndIMU();
    }

    public void setAlliance(Alliance newAlliance) {
        this.alliance = newAlliance;
        this.audienceCellUp = (newAlliance == Alliance.RED);
    }

    public void applyStartPose() {
        if (pinpoint == null) return;
        if (alliance == Alliance.RED) {
            setFieldPose(RED_START_X_IN, RED_START_Y_IN, RED_START_HEADING_DEG);
        } else {
            setFieldPose(BLUE_START_X_IN, BLUE_START_Y_IN, BLUE_START_HEADING_DEG);
        }
    }

    public void setFieldPose(double xInches, double yInches, double headingDegrees) {
        if (pinpoint == null) return;
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, xInches, yInches,
                AngleUnit.DEGREES, headingDegrees
        ));
    }

    /** Flip between Audience CELL and Scoring CELL when the HIVE tips. */
    public void toggleCell() {
        audienceCellUp = !audienceCellUp;
    }

    /** Call once per loop. */
    public void update() {
        if (pinpoint == null) {
            ready = false;
            return;
        }
        pinpoint.update();
        ready = (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY);
        if (!ready) return;

        Pose2D pose = pinpoint.getPosition();
        if (pose == null) return;

        double rawX = pose.getX(DistanceUnit.INCH);
        double rawY = pose.getY(DistanceUnit.INCH);
        double rawH = pose.getHeading(AngleUnit.DEGREES);
        if (Double.isNaN(rawX) || Double.isNaN(rawY) || Double.isNaN(rawH)) return;

        xIn = rawX;
        yIn = rawY;
        headingDeg = rawH;

        double hRad = Math.toRadians(headingDeg);
        double turretX = xIn + TURRET_OFFSET_FORWARD_IN * Math.cos(hRad) - TURRET_OFFSET_LEFT_IN * Math.sin(hRad);
        double turretY = yIn + TURRET_OFFSET_FORWARD_IN * Math.sin(hRad) + TURRET_OFFSET_LEFT_IN * Math.cos(hRad);

        double dx = getActiveCellX() - turretX;
        double dy = getActiveCellY() - turretY;

        distanceToCellIn = Math.hypot(dx, dy);
        double fieldAngleToCellDeg = Math.toDegrees(Math.atan2(dy, dx));
        targetTurretAngleDeg = wrapDeg(fieldAngleToCellDeg - headingDeg + SHOT_OFFSET_DEG);
    }

    private double getActiveCellX() {
        if (alliance == Alliance.RED) return audienceCellUp ? RED_AUDIENCE_X : RED_SCORING_X;
        return audienceCellUp ? BLUE_AUDIENCE_X : BLUE_SCORING_X;
    }

    private double getActiveCellY() {
        if (alliance == Alliance.RED) return audienceCellUp ? RED_AUDIENCE_Y : RED_SCORING_Y;
        return audienceCellUp ? BLUE_AUDIENCE_Y : BLUE_SCORING_Y;
    }

    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    public boolean isReady() { return ready; }
    public double getXIn() { return xIn; }
    public double getYIn() { return yIn; }
    public double getHeadingDeg() { return headingDeg; }
    public double getDistanceToCellIn() { return distanceToCellIn; }
    public double getTargetTurretAngleDeg() { return targetTurretAngleDeg; }
    public String getActiveCellName() { return alliance.name() + " " + (audienceCellUp ? "AUDIENCE" : "SCORING"); }
    public String status() { return pinpoint == null ? configProblem : pinpoint.getDeviceStatus().toString(); }
}
