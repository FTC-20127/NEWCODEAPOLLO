package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret driven by two Axon MAX CRServos (turretA, turretB) + AndyMark absolute encoder (turretEncoder).
 * Aims to a robot-relative angle (+ = LEFT) from Pinpoint, with ±TRAVEL_DEG wire-limit protection.
 */
public class Turret {

    // ---------- hardware settings (verify in Step 1) ----------
    public static boolean USE_ENCODER = true;
    public static int SERVO_DIR = 1;                              // +1 = positive power turns turret LEFT
    public static int SERVO_B_DIR = 1;                            // 1 = same as turretA, -1 = opposite (used by AxonZeroTurret)
    public static int ENCODER_DIR = 1;                            // +1 = volts rise when turning LEFT
    public static double ENCODER_TURNS_PER_TURRET_TURN = 200.0 / 64.0;
    public static double HOME_VOLTAGE = Double.NaN;               // NaN = zero wherever turret points at START
    public static double FULL_TURN_VOLTAGE = 0.0;                 // 0 = use port max (3.3 V)
    public static double TRAVEL_DEG = 180.0;                      // wire limit each way from INIT

    // ---------- PD tuning (verify in Step 3) ----------
    public static double ODOM_KP = 0.012;
    public static double ODOM_KD = 0.0006;
    public static double MIN_POWER = 0.08;
    public static double TOLERANCE_DEG = 1.5;
    public static double LOCK_BAND_DEG = 1.0;
    public static double HOME_KP = 0.01;
    public static double MAX_POWER = 1.0;

    private final CRServo servoA;
    private final CRServo servoB;
    private final AnalogInput encoder;

    private enum Mode { IDLE, HOME, ODOMETRY_AIM }
    private Mode mode = Mode.IDLE;
    private boolean started = false;

    private double targetTurretDeg = 0.0;
    private double lastErrorDeg = Double.NaN;
    private long lastUpdateNs = 0;
    private boolean onTarget = false;
    private double lastPower = 0.0;
    private String note = "";

    private double volts = Double.NaN;
    private double lastRawDeg = Double.NaN;
    private int wraps = 0;
    private double encoderDeg = 0.0;
    private double zeroEncoderDeg = 0.0;
    private double angleDeg = 0.0;
    private double startAngleDeg = 0.0;

    public Turret(HardwareMap hardwareMap) {
        servoA = hardwareMap.get(CRServo.class, "turretA");
        servoB = hardwareMap.get(CRServo.class, "turretB");
        AnalogInput enc = null;
        if (USE_ENCODER) {
            try {
                enc = hardwareMap.get(AnalogInput.class, "turretEncoder");
            } catch (RuntimeException e) {
                enc = null;
            }
        }
        encoder = enc;
    }

    private boolean encoderOn() { return USE_ENCODER && encoder != null; }

    public void init() {
        if (!encoderOn()) { angleDeg = 0; startAngleDeg = 0; return; }
        wraps = 0;
        lastRawDeg = Double.NaN;
        readEncoder();
        if (!Double.isNaN(HOME_VOLTAGE)) {
            zeroEncoderDeg = HOME_VOLTAGE / fullTurnVolts() * 360.0;
            while (encoderDeg - zeroEncoderDeg > 180.0) zeroEncoderDeg += 360.0;
            while (encoderDeg - zeroEncoderDeg < -180.0) zeroEncoderDeg -= 360.0;
        } else {
            zeroEncoderDeg = encoderDeg; // starts at 0.0 deg in INIT so hand-turn test is easy to read
        }
        angleDeg = ENCODER_DIR * (encoderDeg - zeroEncoderDeg) / ENCODER_TURNS_PER_TURRET_TURN;
        startAngleDeg = angleDeg;
    }

    public void start() {
        started = true;
        if (encoderOn() && Double.isNaN(HOME_VOLTAGE)) {
            zeroEncoderDeg = encoderDeg; // straight ahead at START = 0 deg
            startAngleDeg = 0.0;
        }
        lastErrorDeg = Double.NaN;
        lastUpdateNs = System.nanoTime();
    }

    /** Aim the turret to a robot-relative angle (+ = LEFT) from Pinpoint. */
    public void aimToAngle(double desiredTurretDeg) {
        this.mode = Mode.ODOMETRY_AIM;
        this.targetTurretDeg = desiredTurretDeg;
    }

    public void home() {
        mode = Mode.HOME;
        lastErrorDeg = Double.NaN;
    }

    public void idle() {
        mode = Mode.IDLE;
        lastErrorDeg = Double.NaN;
    }

    // Legacy compatibility stubs so any old OpModes in your project still compile cleanly
    public void track(double tagTxDeg, double robotTurnRateDps, double robotHeadingDeg) {
        home();
    }

    public void forgetTagDirection() {
        // no-op (Limelight removed)
    }

    public void update() {
        long now = System.nanoTime();
        double dt = (lastUpdateNs == 0) ? 0.0 : (now - lastUpdateNs) / 1e9;
        lastUpdateNs = now;

        if (encoderOn()) {
            readEncoder();
            angleDeg = ENCODER_DIR * (encoderDeg - zeroEncoderDeg) / ENCODER_TURNS_PER_TURRET_TURN;
        }

        double power = 0.0; // + = turret LEFT

        if (!encoderOn()) {
            note = "NO turretEncoder in config!";
        } else if (mode == Mode.ODOMETRY_AIM) {
            double goalDeg = nearestReachable(targetTurretDeg);
            double error = goalDeg - angleDeg;

            double errorRate = 0.0;
            if (!Double.isNaN(lastErrorDeg) && dt > 0.001 && dt < 0.2) {
                errorRate = (error - lastErrorDeg) / dt;
            }
            lastErrorDeg = error;

            if (Math.abs(error) <= TOLERANCE_DEG) onTarget = true;
            else if (Math.abs(error) > TOLERANCE_DEG + LOCK_BAND_DEG) onTarget = false;

            if (!onTarget) {
                power = ODOM_KP * error + ODOM_KD * errorRate;
                if (Math.signum(power) == Math.signum(error) && Math.abs(power) < MIN_POWER) {
                    power = Math.copySign(MIN_POWER, power);
                }
            }
            note = onTarget ? "on target" : String.format("aiming -> %.1f°", goalDeg);
        } else if (mode == Mode.HOME) {
            double error = nearestReachable(0.0) - angleDeg;
            if (Math.abs(error) > 2.0) {
                power = HOME_KP * error;
                if (Math.abs(power) < MIN_POWER) power = Math.copySign(MIN_POWER, power);
            }
            note = "home (straight ahead)";
        } else {
            note = "idle";
        }

        // Wire limits: never push past ±TRAVEL_DEG from starting angle
        if (encoderOn()) {
            double fromStart = angleDeg - startAngleDeg;
            if (fromStart >= TRAVEL_DEG && power > 0) { power = 0; note = "at LEFT limit"; }
            if (fromStart <= -TRAVEL_DEG && power < 0) { power = 0; note = "at RIGHT limit"; }
        }

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        lastPower = started ? power : 0.0;
        if (started) {
            double a = power * SERVO_DIR;
            servoA.setPower(a);
            servoB.setPower(a * SERVO_B_DIR);
        }
    }

    public void stop() {
        servoA.setPower(0);
        servoB.setPower(0);
    }

    private void readEncoder() {
        volts = encoder.getVoltage();
        double raw = volts / fullTurnVolts() * 360.0;
        if (!Double.isNaN(lastRawDeg)) {
            if (raw - lastRawDeg < -180.0) wraps++;
            else if (raw - lastRawDeg > 180.0) wraps--;
        }
        lastRawDeg = raw;
        encoderDeg = raw + 360.0 * wraps;
    }

    private double nearestReachable(double deg) {
        double best = Double.NaN;
        for (int k = -3; k <= 3; k++) {
            double c = deg + 360.0 * k;
            if (Math.abs(c - startAngleDeg) > TRAVEL_DEG) continue;
            if (Double.isNaN(best) || Math.abs(c - angleDeg) < Math.abs(best - angleDeg)) best = c;
        }
        if (Double.isNaN(best)) {
            best = Math.max(startAngleDeg - TRAVEL_DEG, Math.min(startAngleDeg + TRAVEL_DEG, deg));
        }
        return best;
    }

    private double fullTurnVolts() {
        if (encoder == null) return 3.3;
        double v = FULL_TURN_VOLTAGE > 0 ? FULL_TURN_VOLTAGE : encoder.getMaxVoltage();
        return v > 0 ? v : 3.3;
    }

    public double angleDeg() { return angleDeg; }
    public double angleFromStartDeg() { return angleDeg - startAngleDeg; }
    public double targetAngleDeg() { return targetTurretDeg; }
    public double encoderVolts() { return volts; }
    public boolean homeSet() { return !Double.isNaN(HOME_VOLTAGE); }
    public double power() { return lastPower; }
    public String note() { return note; }
}
