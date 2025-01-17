package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TelemetryLogger {


    public static ArrayList<Double> loggedTimes = new ArrayList<Double>();
    public static ArrayList<String> loggedMessages = new ArrayList<String>();
    private static ElapsedTime logTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static void log(Object message) {
        loggedTimes.add(logTimer.time());
        logTimer.reset();

        String finalMessage;
        if (message instanceof Double) { // rounds a double if it is a double
            finalMessage = String.valueOf(functions.round((Double) message, 4));
        } else {
            finalMessage = String.valueOf(message);
        }

        loggedMessages.add(finalMessage);
    }

    public static void log() {
        log("");
    }

    public static void log(String caption, Object data) {
        String finalCaption, finalData;
        if (data instanceof Double) { // rounds a double if it is a double
            finalData = String.valueOf(functions.round((Double) data, 4));
        } else {
            finalData = String.valueOf(data);
        }

        if (caption.endsWith(":")) { // adds a ":" if there isn't one already and also a space between the data
            finalCaption = caption + " ";
        } else {
            finalCaption = caption + ": ";
        }

        log(finalCaption + finalData);
    }

    public static void logReset() {
        loggedTimes.clear();
        loggedMessages.clear();
        logTimer.reset();
    }

    public static void addLogsToTelemetry(Telemetry telemetry) {
        if (loggedTimes.isEmpty()) telemetry.addLine("Nothing logged");
        else {
            for (int i = 0; i < loggedTimes.size(); i++) {
                telemetry.addData("Log " + (i + 1) + ":" + (Math.round(loggedTimes.get(i)) / 1000) + " - ", loggedMessages.get(i));
            }
        }
        logReset();
    }
}
