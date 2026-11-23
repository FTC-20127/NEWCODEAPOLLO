package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * MAIN TeleOp: Control Hub IMU drive (UNCHANGED) + Pinpoint Limelight-free tracking
 * + RPM-only Pollen shooting (hood held at one fixed position).
 *
 * BEFORE PRESSING START (in INIT):
 *   D-pad Left / Right    : Select RED / BLUE alliance
 *   Hold still until Pinpoint says READY
 *
 * GAMEPAD 1:
 *   Left stick            : drive (field relative)
 *   Right stick X         : turn
 *   Right stick press     : switch robot relative / field relative
 *   Left stick press      : slow mode on/off
 *   X                     : reset Control Hub IMU drive heading
 *   Right trigger (hold)  : shooter spin up
 *   Left trigger (hold)   : intake + passthrough forward (hold with RT to shoot)
 *   Left bumper (hold)    : intake + passthrough reverse
 *   A                     : flip active HIVE CELL (Audience <-> Scoring when HIVE tips)
 *   B                     : turret tracking on/off (starts OFF for safety while testing)
 *   Back                  : reset Pinpoint pose to match start pose
 *   D-pad Up / Down       : +50 / -50 RPM trim (only when TUNE_MODE = true)
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleop extends OpMode {

    public static double DRIVE_ANGLE_OFFSET_DEG = 0;

    public static double MAX_SPEED = 1.0;        // lower this for outreach events
    public static double SLOW_MODE_SCALE = 0.4;  // drive speed while slow mode is on

    // ---------- mechanism settings ----------
    public static double SHOOTER_MAX_RPM = 7750.0;
    public static double SHOOTER_TICKS_PER_REV = 28.0;
    public static double SHOOTER_KF = 1.0 / 6000.0;
    public static double SHOOTER_KP = 0.0004;
    public static double SHOOTER_KI = 0.0001;
    public static double SHOOTER_FILTER = 0.6;

    // burst settings
    public static double SHOOTER_FEED_BOOST = 0.15;
    public static double SHOOTER_BANG_BAND  = 150;

    public static double INTAKE_POWER = 1.0;
    public static double PASSTHROUGH_POWER = 1.0;
    public static double TRIGGER_DEADZONE = 0.1;

    // Fixed hood position (RPM-only for now; holds hood firmly against Pollen)
    public static double HOOD_FIXED_POSITION = 0.40;
    public static double HOOD_FEED_PUSH      = 0.02;

    /** Set true during Step 4 to live-tune RPM with D-pad Up/Down; false for matches. */
    public static boolean TUNE_MODE = false;

    /** Set true once Steps 1-3 pass so tracking turns on automatically at START. */
    public static boolean START_WITH_TRACKING_ON = false;

    public static PinpointFieldTracker.Alliance DEFAULT_ALLIANCE = PinpointFieldTracker.Alliance.BLUE;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    IMU imu;

    DcMotor intake;
    DcMotor passthrough;
    DcMotorEx shooterLeft;
    DcMotorEx shooterRight;
    Servo hood;

    PinpointFieldTracker tracker;
    Turret turret;
    PinpointFieldTracker.Alliance alliance;

    boolean shooterOn = false;
    boolean lastA = false;
    boolean tracking = false, lastB = false;
    boolean lastBack = false;
    boolean lastDpadUp = false, lastDpadDown = false;

    double filteredRpm = 0;
    double shooterIntegral = 0;
    double lastLoopTime = 0;
    double shooterPower = 0;
    double hoodTarget = HOOD_FIXED_POSITION;
    boolean slowMode = false, lastLeftStickBtn = false;
    boolean robotRelative = false, lastRightStickBtn = false;

    // RPM table + burst lock
    double totalTargetRpm = SHOOTER_MAX_RPM;
    double burstTotalRpm = SHOOTER_MAX_RPM;
    boolean wasFeeding = false;
    double tuneRpmOffset = 0.0;
    boolean startPoseApplied = false;

    // Control Hub IMU values (for driving — UNCHANGED)
    double headingDeg = 0;
    double headingRad = 0;
    double loopMs = 0;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        intake       = hardwareMap.get(DcMotor.class,   "intake");
        passthrough  = hardwareMap.get(DcMotor.class,   "passthrought");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooter_left");
        shooterRight = hardwareMap.get(DcMotorEx.class, "Shooter_right");
        hood         = hardwareMap.get(Servo.class,     "hood");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        passthrough.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ---------- Pinpoint odometry + turret (NO Limelight) ----------
        alliance = DEFAULT_ALLIANCE;
        tracker = new PinpointFieldTracker(hardwareMap);
        turret = new Turret(hardwareMap);
        tracker.init(alliance);
        turret.init();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left) {
            alliance = PinpointFieldTracker.Alliance.RED;
            tracker.setAlliance(alliance);
        } else if (gamepad1.dpad_right) {
            alliance = PinpointFieldTracker.Alliance.BLUE;
            tracker.setAlliance(alliance);
        }

        tracker.update();
        turret.update();

        telemetry.addData("Alliance (D-pad L=RED / R=BLUE)", alliance);
        telemetry.addData("Starting CELL", tracker.getActiveCellName());
        telemetry.addData("Pinpoint status", tracker.status());
        telemetry.addData("Turret encoder volts", "%.3f", turret.encoderVolts());
        telemetry.addData("Turret angle", "%.1f°", turret.angleDeg());
        telemetry.addData("Turret note", turret.note());
    }

    @Override
    public void start() {
        imu.resetYaw();
        turret.start();
        tracking = START_WITH_TRACKING_ON;
        startPoseApplied = false;
        if (tracker.isReady()) {
            tracker.applyStartPose();
            startPoseApplied = true;
        }

        hoodTarget = HOOD_FIXED_POSITION;
        hood.setPosition(hoodTarget);

        lastLoopTime = getRuntime();
    }

    @Override
    public void loop() {
        // ---------- timing ----------
        double now = getRuntime();
        double rawDt = now - lastLoopTime;
        double dt = Math.max(0.0, Math.min(0.1, rawDt));
        lastLoopTime = now;
        loopMs = rawDt * 1000.0;

        // ---------- Control Hub IMU + drive (UNCHANGED) ----------
        if (gamepad1.x) { imu.resetYaw(); }
        headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        headingRad = Math.toRadians(headingDeg);

        if (gamepad1.left_stick_button && !lastLeftStickBtn) slowMode = !slowMode;
        lastLeftStickBtn = gamepad1.left_stick_button;
        if (gamepad1.right_stick_button && !lastRightStickBtn) robotRelative = !robotRelative;
        lastRightStickBtn = gamepad1.right_stick_button;

        double forward = -gamepad1.left_stick_y;
        double right   = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;
        double speed   = slowMode ? MAX_SPEED * SLOW_MODE_SCALE : MAX_SPEED;

        if (robotRelative) {
            drive(forward, right, rotate, speed);
        } else {
            driveFieldRelative(forward, right, rotate, speed);
        }

        // ---------- Pinpoint update + CELL flip (A) + pose reset (Back) ----------
        tracker.update();
        if (!startPoseApplied && tracker.isReady()) {
            tracker.applyStartPose();
            startPoseApplied = true;
        }
        if (gamepad1.a && !lastA) tracker.toggleCell();
        lastA = gamepad1.a;

        if (gamepad1.back && !lastBack && tracker.isReady()) tracker.applyStartPose();
        lastBack = gamepad1.back;

        if (TUNE_MODE) {
            if (gamepad1.dpad_up && !lastDpadUp) tuneRpmOffset += 50.0;
            if (gamepad1.dpad_down && !lastDpadDown) tuneRpmOffset -= 50.0;
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // ---------- intake + passthrough: LT = both forward, LB = both reverse ----------
        boolean reversingBoth = gamepad1.left_bumper;
        boolean feedingForward = !reversingBoth && (gamepad1.left_trigger > TRIGGER_DEADZONE);

        if (reversingBoth) {
            intake.setPower(-INTAKE_POWER);
            passthrough.setPower(-PASSTHROUGH_POWER);
        } else if (feedingForward) {
            intake.setPower(INTAKE_POWER);
            passthrough.setPower(PASSTHROUGH_POWER);
        } else {
            intake.setPower(0);
            passthrough.setPower(0);
        }

        // ---------- shooter: RT (hold) runs flywheel with RPM-only lookup ----------
        shooterOn = gamepad1.right_trigger > TRIGGER_DEADZONE;
        boolean feeding = shooterOn && feedingForward;

        double leftRpmNow  = Math.abs(shooterLeft.getVelocity())  / SHOOTER_TICKS_PER_REV * 60.0;
        double rightRpmNow = Math.abs(shooterRight.getVelocity()) / SHOOTER_TICKS_PER_REV * 60.0;
        double measuredRpm = Math.max(leftRpmNow, rightRpmNow);
        filteredRpm += SHOOTER_FILTER * (measuredRpm - filteredRpm);

        double distIn = tracker.getDistanceToCellIn();
        totalTargetRpm = SHOOTER_MAX_RPM;
        if (tracker.isReady() && !Double.isNaN(distIn)) {
            double tableRpm = PollenRpmTable.totalRpmFor(distIn);
            if (!Double.isNaN(tableRpm)) totalTargetRpm = tableRpm;
        }
        totalTargetRpm = Math.max(0.0, Math.min(11500.0, totalTargetRpm + tuneRpmOffset));

        if (feeding && !wasFeeding) burstTotalRpm = totalTargetRpm;
        if (feeding) totalTargetRpm = burstTotalRpm;
        wasFeeding = feeding;

        double targetRpm = totalTargetRpm / 2.0;
        if (shooterOn) {
            double error = targetRpm - filteredRpm;
            shooterIntegral += error * dt;
            double maxIntegral = 0.3 / SHOOTER_KI;
            shooterIntegral = Math.max(-maxIntegral, Math.min(maxIntegral, shooterIntegral));

            shooterPower = SHOOTER_KF * targetRpm + SHOOTER_KP * error + SHOOTER_KI * shooterIntegral;
            if (feeding) shooterPower += SHOOTER_FEED_BOOST;
            if (error > SHOOTER_BANG_BAND) shooterPower = 1.0;
            shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));
        } else {
            shooterPower = 0;
            shooterIntegral = 0;
        }
        shooterLeft.setPower(shooterPower);
        shooterRight.setPower(shooterPower);

        // ---------- fixed hood ----------
        hoodTarget = HOOD_FIXED_POSITION + (feeding ? HOOD_FEED_PUSH : 0.0);
        hoodTarget = Math.max(0.0, Math.min(1.0, hoodTarget));
        hood.setPosition(hoodTarget);

        // ---------- turret: B toggles tracking ----------
        if (gamepad1.b && !lastB) tracking = !tracking;
        lastB = gamepad1.b;

        if (tracking && tracker.isReady()) {
            turret.aimToAngle(tracker.getTargetTurretAngleDeg());
        } else {
            turret.home();
        }
        turret.update();

        // ---------- telemetry ----------
        telemetry.addData("Loop", "%.0f ms | Drive %s | IMU %.1f°",
                loopMs, robotRelative ? "ROBOT" : "FIELD", headingDeg);
        telemetry.addData("Pinpoint", "%s | X=%.1f Y=%.1f H=%.1f°",
                tracker.status(), tracker.getXIn(), tracker.getYIn(), tracker.getHeadingDeg());
        telemetry.addData("CELL (A flips)", "%s | Dist %.1f in", tracker.getActiveCellName(), distIn);
        telemetry.addData("Turret (B toggles)", "%s | cur %.1f° -> tgt %.1f° (pow %+.2f) [%s]",
                tracking ? "TRACK" : "HOME",
                turret.angleDeg(), turret.targetAngleDeg(), turret.power(), turret.note());
        telemetry.addData("Shooter (RT)", "%s | %.0f / %.0f RPM",
                shooterOn ? (feeding ? "FEEDING" : "ON") : "off",
                filteredRpm * 2.0, shooterOn ? totalTargetRpm : 0.0);
        telemetry.addData("Intake+Pass (LT/LB)", reversingBoth ? "REVERSE" : (feedingForward ? "FORWARD" : "off"));
        if (TUNE_MODE) {
            telemetry.addData("WRITE DOWN", "{ %.1f, %.0f },", distIn, totalTargetRpm);
        }
    }

    @Override
    public void stop() {
        drive(0, 0, 0, 0);
        intake.setPower(0);
        passthrough.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        turret.stop();
    }

    private void driveFieldRelative(double forward, double right, double rotate, double speed) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta + headingRad);

        drive(r * Math.sin(theta), r * Math.cos(theta), rotate, speed);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate, double speed) {
        if (DRIVE_ANGLE_OFFSET_DEG != 0) {
            double a = Math.toRadians(DRIVE_ANGLE_OFFSET_DEG);
            double f = forward * Math.cos(a) + right * Math.sin(a);
            double r = right * Math.cos(a) - forward * Math.sin(a);
            forward = f;
            right = r;
        }

        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower  = forward + right - rotate;
        double backLeftPower   = forward - right + rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(speed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(speed * (frontRightPower / maxPower));
        backLeftDrive.setPower(speed * (backLeftPower / maxPower));
        backRightDrive.setPower(speed * (backRightPower / maxPower));
    }
}
