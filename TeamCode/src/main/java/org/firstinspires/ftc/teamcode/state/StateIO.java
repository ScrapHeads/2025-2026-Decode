package org.firstinspires.ftc.teamcode.state;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.io.File;

/**
 * Utility class for saving and loading {@link RobotState} objects
 * between Autonomous and TeleOp. The state is persisted to a JSON
 * file on the Robot Controller and can be read back later.
 *
 * <p>File location: /sdcard/FIRST/settings/auto_handoff.json</p>
 */
public class StateIO {

    // Prevent instantiation
    private StateIO() {}

    private static final String FILENAME = "RobotState.json";  // corrected extension
    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    /**
     * Gets the file reference for storing the robot state.
     *
     * @return the File object pointing to RobotState.json in the FIRST settings directory
     */
    private static File getFile() {
        return AppUtil.getInstance().getSettingsFile(FILENAME);
    }

    /**
     * Saves the given RobotState to disk as JSON.
     * If the state is null or an error occurs, nothing is written.
     *
     */
    public static void save() {
        try {
            File file = getFile();
            String json = GSON.toJson(RobotState.getInstance());

            ReadWriteFile.writeFile(file, json);

        } catch (Exception e) {
            tele.addLine("save failed");
            tele.update();
        }
    }

    /**
     * Loads a RobotState from disk.
     *
     * @return the deserialized RobotState, or null if the file is missing
     *         or could not be parsed
     */
    public static void load() {
        try {
            File file = getFile();
            if (file == null || !file.exists()) return;

            String json = ReadWriteFile.readFile(file);
            if (json == null || json.isEmpty()) return;

            // Create a temp container to transfer data
            RobotState.getInstance().setAll(GSON.fromJson(json, RobotState.class));

        } catch (Exception e) {
            // Possibly add a mock RobotState file update
            RobotState.getInstance().setAll(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    null,
                    new BallColor[] {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY},
                    new ChassisSpeeds()
            );
            tele.addLine("Failed to load");
            tele.update();
        }
    }

    /**
     * Clears the saved RobotState so it cannot be reused accidentally.
     * Writes an empty JSON object ("{}") into the file.
     */
    public static void clear() {
        try {
            ReadWriteFile.writeFile(getFile(), "{}");
        } catch (Exception ignored) {
            // Safe to ignore
        }
    }
}
