package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/*
 * BiobuzzHuskyLens
 * ----------------
 * Helper class that wraps the FTC SDK's built-in DFRobot HuskyLens driver
 * (com.qualcomm.hardware.dfrobot.HuskyLens) and turns its raw "blocks" into
 * BIOBUZZ (2026-2027) HIVE CELL AprilTag sightings.
 *
 * BIOBUZZ AprilTag facts (Competition Manual, Section 9):
 *   - Family 36h11, 3.25 in tags.
 *   - Four 4-tag clusters, one on the BOTTOM of each HIVE CELL, facing DOWN
 *     toward the tiles. They move with the tipping CELLS (not fixed wall targets).
 *       IDs 30-33  Red  CELL, side opposite the audience
 *       IDs 34-37  Red  CELL, audience side
 *       IDs 38-41  Blue CELL, audience side
 *       IDs 42-45  Blue CELL, side opposite the audience
 *   - Because the tags face down, the HuskyLens must be mounted looking UP.
 *
 * IMPORTANT - how HuskyLens (V1) reports tag IDs:
 *   In TAG_RECOGNITION mode the HuskyLens does NOT report the printed AprilTag
 *   number. It reports a "learned ID" (1, 2, 3, ...) in the order you taught it
 *   tags on the device, and ID 0 for any tag it sees but was never taught.
 *   So you must teach it all 16 BIOBUZZ tags, and LEARNED_TO_FIELD below maps
 *   learned IDs back to field IDs. See the setup notes in HuskyLensBiobuzzTest.
 *
 * Hardware config: add the device as type "HuskyLens" on an I2C bus of the
 * Control/Expansion Hub, named "huskylens" (or pass your own name).
 */
public class HuskyLensCamera {

    // ---------------------------------------------------------------- field data

    public enum Alliance { RED, BLUE }

    /** The four HIVE CELLS and their AprilTag clusters. */
    public enum Cell {
        RED_FAR  (Alliance.RED,  30, "Red CELL, far side"),
        RED_NEAR (Alliance.RED,  34, "Red CELL, audience side"),
        BLUE_NEAR(Alliance.BLUE, 38, "Blue CELL, audience side"),
        BLUE_FAR (Alliance.BLUE, 42, "Blue CELL, far side");

        public final Alliance alliance;
        public final int firstTagId;      // cluster is firstTagId .. firstTagId + 3
        public final String label;

        Cell(Alliance alliance, int firstTagId, String label) {
            this.alliance = alliance;
            this.firstTagId = firstTagId;
            this.label = label;
        }

        public boolean contains(int fieldTagId) {
            return fieldTagId >= firstTagId && fieldTagId <= firstTagId + 3;
        }

        /** Returns the CELL a field tag belongs to, or null if it isn't a BIOBUZZ tag. */
        public static Cell forTag(int fieldTagId) {
            for (Cell c : values()) {
                if (c.contains(fieldTagId)) return c;
            }
            return null;
        }
    }

    /** Printed BIOBUZZ tag size (black square edge), inches. */
    public static final double TAG_SIZE_IN = 3.25;

    // ------------------------------------------------------------ camera constants

    /** HuskyLens image is 320 x 240 pixels, centre at (160, 120). */
    public static final int SCREEN_WIDTH  = 320;
    public static final int SCREEN_HEIGHT = 240;
    public static final int CENTER_X = SCREEN_WIDTH / 2;
    public static final int CENTER_Y = SCREEN_HEIGHT / 2;

    /*
     * Focal length in pixels, used for bearing and distance estimates.
     * ~277 assumes a ~60 degree horizontal field of view. CALIBRATE IT:
     *   hold a BIOBUZZ tag square-on at a measured distance D (inches),
     *   read the tag width W (pixels) from the test OpMode telemetry, then
     *   FOCAL_LENGTH_PX = W * D / TAG_SIZE_IN.
     */
    public static double FOCAL_LENGTH_PX = 277.0;

    /*
     * Learned ID -> printed field ID. Index = learned ID reported by HuskyLens.
     * Index 0 is "seen but not learned" and maps to -1.
     *
     * This default assumes you taught the tags IN ORDER 30, 31, 32, ... 45,
     * so learned ID 1 = tag 30, learned ID 16 = tag 45.
     * If you taught them in a different order, edit this table to match.
     */
    private static final int[] LEARNED_TO_FIELD = {
            -1,                         // 0  = unlearned tag
            30, 31, 32, 33,             // 1-4   Red  far
            34, 35, 36, 37,             // 5-8   Red  audience side
            38, 39, 40, 41,             // 9-12  Blue audience side
            42, 43, 44, 45              // 13-16 Blue far
    };

    // ---------------------------------------------------------------- result types

    /** One tag the HuskyLens can see right now. */
    public static class TagSighting {
        public final int learnedId;
        public final int fieldId;       // -1 if unlearned / unmapped
        public final Cell cell;         // null if not a BIOBUZZ tag
        public final int x, y;          // centre of bounding box, pixels
        public final int width, height; // bounding box size, pixels

        TagSighting(HuskyLens.Block b) {
            learnedId = b.id;
            fieldId = (b.id >= 0 && b.id < LEARNED_TO_FIELD.length) ? LEARNED_TO_FIELD[b.id] : -1;
            cell = Cell.forTag(fieldId);
            x = b.x;
            y = b.y;
            width = b.width;
            height = b.height;
        }

        /** Horizontal offset from image centre, pixels (+ = right). */
        public int xErrorPx() { return x - CENTER_X; }

        /** Vertical offset from image centre, pixels (+ = down in the image). */
        public int yErrorPx() { return y - CENTER_Y; }

        /** Horizontal angle from camera axis to the tag, degrees (+ = right). */
        public double bearingDeg() {
            return Math.toDegrees(Math.atan2(xErrorPx(), FOCAL_LENGTH_PX));
        }

        /**
         * Rough straight-line range to the tag, inches, from its apparent size.
         * Uses the larger box side so viewing the tag at an angle hurts less.
         * Only as good as FOCAL_LENGTH_PX; treat it as an estimate.
         */
        public double rangeIn() {
            int sizePx = Math.max(width, height);
            return sizePx > 0 ? TAG_SIZE_IN * FOCAL_LENGTH_PX / sizePx : Double.NaN;
        }

        @Override
        public String toString() {
            return String.format("tag %s (learned %d) @(%d,%d) %dx%d  bearing %.1f°  ~%.0f in",
                    fieldId >= 0 ? String.valueOf(fieldId) : "?", learnedId,
                    x, y, width, height, bearingDeg(), rangeIn());
        }
    }

    /** Every tag currently visible from one CELL's cluster, combined. */
    public static class CellSighting {
        public final Cell cell;
        public final List<TagSighting> tags;

        CellSighting(Cell cell, List<TagSighting> tags) {
            this.cell = cell;
            this.tags = Collections.unmodifiableList(tags);
        }

        /** Average centre X of the visible tags in this cluster, pixels. */
        public double centerX() {
            double sum = 0;
            for (TagSighting t : tags) sum += t.x;
            return sum / tags.size();
        }

        public double centerY() {
            double sum = 0;
            for (TagSighting t : tags) sum += t.y;
            return sum / tags.size();
        }

        public double xErrorPx() { return centerX() - CENTER_X; }

        public double bearingDeg() {
            return Math.toDegrees(Math.atan2(xErrorPx(), FOCAL_LENGTH_PX));
        }

        /** Average range estimate across visible tags, inches. */
        public double rangeIn() {
            double sum = 0;
            int n = 0;
            for (TagSighting t : tags) {
                double r = t.rangeIn();
                if (!Double.isNaN(r)) { sum += r; n++; }
            }
            return n > 0 ? sum / n : Double.NaN;
        }

        /** Largest visible tag size; bigger = closer. Used to pick the nearest CELL. */
        public int largestTagPx() {
            int best = 0;
            for (TagSighting t : tags) best = Math.max(best, Math.max(t.width, t.height));
            return best;
        }
    }

    // ---------------------------------------------------------------- state

    private final HuskyLens huskyLens;
    private HuskyLens.Block[] lastBlocks = new HuskyLens.Block[0];
    private final List<TagSighting> tags = new ArrayList<>();
    private final Map<Cell, CellSighting> cells = new EnumMap<>(Cell.class);
    private final Map<Cell, Long> lastSeenNanos = new EnumMap<>(Cell.class);
    private int unlearnedCount = 0;
    private long lastUpdateNanos = 0;
    private double loopHz = 0;

    public HuskyLensCamera(HardwareMap hardwareMap) {
        this(hardwareMap, "huskylens");
    }

    public HuskyLensCamera(HardwareMap hardwareMap, String deviceName) {
        huskyLens = hardwareMap.get(HuskyLens.class, deviceName);
    }

    /** True if the HuskyLens answers on the I2C bus. Call during init. */
    public boolean isConnected() {
        return huskyLens.knock();
    }

    /**
     * Puts the HuskyLens into AprilTag mode. The SDK does not choose an
     * algorithm for you, so always call this during init.
     */
    public void start() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    /**
     * Reads the sensor once and rebuilds the sightings. Call once per loop.
     * @return true if at least one BIOBUZZ CELL tag is visible.
     */
    public boolean update() {
        long now = System.nanoTime();
        if (lastUpdateNanos != 0) {
            double dt = (now - lastUpdateNanos) / 1e9;
            if (dt > 0) loopHz = 1.0 / dt;
        }
        lastUpdateNanos = now;

        HuskyLens.Block[] blocks = huskyLens.blocks();
        lastBlocks = (blocks != null) ? blocks : new HuskyLens.Block[0];

        tags.clear();
        cells.clear();
        unlearnedCount = 0;

        Map<Cell, List<TagSighting>> grouped = new EnumMap<>(Cell.class);
        for (HuskyLens.Block b : lastBlocks) {
            TagSighting t = new TagSighting(b);
            tags.add(t);
            if (t.cell == null) {
                unlearnedCount++;
                continue;
            }
            List<TagSighting> list = grouped.get(t.cell);
            if (list == null) {
                list = new ArrayList<>();
                grouped.put(t.cell, list);
            }
            list.add(t);
        }

        for (Map.Entry<Cell, List<TagSighting>> e : grouped.entrySet()) {
            cells.put(e.getKey(), new CellSighting(e.getKey(), e.getValue()));
            lastSeenNanos.put(e.getKey(), now);
        }
        return !cells.isEmpty();
    }

    // ---------------------------------------------------------------- queries

    /** Raw blocks from the most recent update(), for debugging. */
    public HuskyLens.Block[] getRawBlocks() { return lastBlocks; }

    /** All tags seen in the most recent update(). */
    public List<TagSighting> getTags() { return Collections.unmodifiableList(tags); }

    /** Tags the HuskyLens framed but that aren't mapped to a BIOBUZZ ID. */
    public int getUnlearnedCount() { return unlearnedCount; }

    /** Every CELL visible in the most recent update(). */
    public List<CellSighting> getVisibleCells() { return new ArrayList<>(cells.values()); }

    /** A specific CELL if visible right now, else null. */
    public CellSighting getCell(Cell cell) { return cells.get(cell); }

    /** A specific field tag if visible right now, else null. */
    public TagSighting getTag(int fieldTagId) {
        for (TagSighting t : tags) {
            if (t.fieldId == fieldTagId) return t;
        }
        return null;
    }

    /**
     * The nearest visible CELL belonging to an alliance (judged by the
     * largest tag on screen), or null if none of that alliance's CELLS is visible.
     */
    public CellSighting getNearestCell(Alliance alliance) {
        CellSighting best = null;
        for (CellSighting c : cells.values()) {
            if (c.cell.alliance != alliance) continue;
            if (best == null || c.largestTagPx() > best.largestTagPx()) best = c;
        }
        return best;
    }

    /** Milliseconds since a CELL was last seen, or Long.MAX_VALUE if never. */
    public long millisSinceSeen(Cell cell) {
        Long t = lastSeenNanos.get(cell);
        return t == null ? Long.MAX_VALUE : (System.nanoTime() - t) / 1_000_000L;
    }

    /** How often update() is being called, Hz. */
    public double getLoopHz() { return loopHz; }

    /**
     * Simple proportional turn command to centre a CELL in the image.
     * Returns a value in [-maxPower, maxPower]; + means the target is to the
     * right of image centre. Flip the sign if your camera is mounted
     * mirrored or rotated, or if your drive code uses the opposite convention.
     */
    public static double turnPowerToward(CellSighting target, double kP, double maxPower, double deadbandDeg) {
        if (target == null) return 0;
        double err = target.bearingDeg();
        if (Math.abs(err) < deadbandDeg) return 0;
        double p = kP * err;
        return Math.max(-maxPower, Math.min(maxPower, p));
    }
}
