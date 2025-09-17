package org.firstinspires.ftc.teamcode.state;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

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

    private static final String FILENAME = "auto_handoff.json";  // corrected extension
    private static final Gson GSON = new GsonBuilder().create();

    /**
     * Gets the file reference for storing the robot state.
     *
     * @return the File object pointing to auto_handoff.json in the FIRST settings directory
     */
    private static File getFile() {
        return AppUtil.getInstance().getSettingsFile(FILENAME);
    }

    /**
     * Saves the given RobotState to disk as JSON.
     * If the state is null or an error occurs, nothing is written.
     *
     * @param state the RobotState object to save
     */
    public static void save(RobotState state) {
        if (state == null) return;
        try {
            String json = GSON.toJson(state);
            ReadWriteFile.writeFile(getFile(), json);
        } catch (Exception e) {
            // Errors are ignored to avoid crashing an OpMode.
            // Consider logging with telemetry for debugging.
        }
    }

    /**
     * Loads a RobotState from disk.
     *
     * @return the deserialized RobotState, or null if the file is missing
     *         or could not be parsed
     */
    public static RobotState load() {
        try {
            File f = getFile();
            if (f == null || !f.exists()) return null;
            String json = ReadWriteFile.readFile(f);
            if (json == null || json.isEmpty()) return null;
            return GSON.fromJson(json, RobotState.class);
        } catch (Exception ignored) {
            return null;
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
