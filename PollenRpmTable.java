package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.Arrays;

/**
 * POLLEN shooter speed by distance (RPM-only lookup table).
 *
 * Distance is measured along the floor (inches) from the center of the turret
 * to the floor directly under the active HIVE CELL (shown on telemetry as "Dist").
 *
 * To tune: set MainTeleop.TUNE_MODE = true, park at a distance, turn shooter on (Y),
 * adjust with D-pad Up (+50 RPM) / Down (-50 RPM), and copy the "WRITE DOWN" line below.
 *
 * RPM is TOTAL (both shooter motors added together, same scale as SHOOTER_MAX_RPM = 7750).
 */
public final class PollenRpmTable {

    private PollenRpmTable() {
    }

    public static double[][] TABLE = {
            //  distance (in)   total RPM
            {   24,             7750 },
            {   30,             7750 },
            {   36,             7750 },
            {   42,             7750 },
            {   48,             7750 },
            {   54,             7750 },
            {   60,             7750 },
            {   66,             7750 },
            {   72,             7750 },
            {   78,             7750 },
            {   84,             7750 },
            {   90,             7750 },
            {   96,             7750 },
            {  102,             7750 },
            {  108,             7750 },
            {  114,             7750 },
            {  120,             7750 },
            {  126,             7750 },
            {  132,             7750 },
            {  138,             7750 },
            {  144,             7750 },
    };

    /** Total RPM for {@code distanceIn} from the table, or NaN if distance is unknown. */
    public static double totalRpmFor(double distanceIn) {
        if (Double.isNaN(distanceIn) || TABLE.length == 0) return Double.NaN;
        double[][] rows = TABLE.clone();
        Arrays.sort(rows, (a, b) -> Double.compare(a[0], b[0]));
        if (distanceIn <= rows[0][0]) return rows[0][1];
        for (int i = 1; i < rows.length; i++) {
            if (distanceIn <= rows[i][0]) {
                double d0 = rows[i - 1][0], d1 = rows[i][0];
                if (d1 == d0) return rows[i][1];
                double t = (distanceIn - d0) / (d1 - d0);
                return rows[i - 1][1] + t * (rows[i][1] - rows[i - 1][1]);
            }
        }
        return rows[rows.length - 1][1];
    }
}
