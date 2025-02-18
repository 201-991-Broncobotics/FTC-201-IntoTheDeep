package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;

public class FrameRateStabilizer {

    private final ElapsedTime frameRateTimer;
    private double lastWaitTime = 0, minMaxTimeDifference = 0, currentAverageTime = 1;
    public boolean Enabled = true;
    public double lowerFrameRateRatio = 0.75, maxFramesToAverage = 20, minimumFramesToAverage = 5, maxWait = 100, minimumFrameRate = 10; // milliseconds
    private final ArrayList<Double> periodCache = new ArrayList<Double>();


    public FrameRateStabilizer(double lowerFrameRateRatio, double maxFramesToAverage, double maxWait) {
        this.lowerFrameRateRatio = lowerFrameRateRatio;
        this.maxFramesToAverage = maxFramesToAverage;
        this.maxWait = maxWait;
        frameRateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }


    public void stabilize() { // must be called every loop
        correctSettings();

        double currentTime = frameRateTimer.time();
        periodCache.add(currentTime);
        if (periodCache.size() > maxFramesToAverage) periodCache.remove(0);

        if (periodCache.size() >= minimumFramesToAverage) { // enough data to start averaging
            currentAverageTime = periodCache.stream().mapToDouble(a -> a).average().orElse(0.0);
            minMaxTimeDifference = Collections.max(periodCache) - Collections.min(periodCache);

            if (Enabled && 1000 / currentAverageTime > minimumFrameRate && currentTime < currentAverageTime) { // above minimum framerate and is below average framerate time
                lastWaitTime = Math.round(lowerFrameRateRatio * (currentAverageTime - currentTime));
                if (lastWaitTime > maxWait && maxWait > 0) lastWaitTime = maxWait;

                if (lastWaitTime > 0) { // just to make sure the wait time is a valid number
                    Sleep(Math.round(lastWaitTime));
                }
            } else lastWaitTime = 0;

        }

        frameRateTimer.reset();
    }

    private void correctSettings() { // just because if some of these settings are the wrong numbers, bad things will happen
        if (lowerFrameRateRatio > 1) lowerFrameRateRatio = 1; // greater than 1 causes an ever growing wait time
        else if (lowerFrameRateRatio < 0) lowerFrameRateRatio = 0; // idk what this would do
        if (maxFramesToAverage < 2) maxFramesToAverage = 2;
        if (minimumFramesToAverage < 2) minimumFramesToAverage = 2;
        if (minimumFramesToAverage > maxFramesToAverage) minimumFramesToAverage = maxFramesToAverage;
        if (maxWait < 0) maxWait = 0; // 0 just makes there be no max wait time
        if (minimumFrameRate < 0) minimumFrameRate = 0;


    }

    public static void Sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            // Wait the set amount of time in milliseconds
        }
    }

    public void enable() { Enabled = true; }
    public void disable() { Enabled = false; }
    public boolean isEnabled() { return Enabled; }
    public double getAverageTime() { return currentAverageTime; }
    public double getAverageFrameRate() { return 1000 / currentAverageTime; }
    public double getCurrentWaitTime() { return lastWaitTime; } // milliseconds
    public double getMinMaxTimeDifference() { return minMaxTimeDifference; } // milliseconds
    public int getCacheSize() { return periodCache.size(); }

}
