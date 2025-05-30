package org.firstinspires.ftc.teamcode.robots.deepthought.util;

import android.content.SharedPreferences;

import com.google.gson.Gson;

import org.firstinspires.ftc.teamcode.RC;


public class PositionCache {
    private int updateInterval;
    SharedPreferences sharedPref = RC.a().getPreferences(RC.c().MODE_PRIVATE);
    SharedPreferences.Editor editor = sharedPref.edit();
    Gson gson = new Gson();
    private int cyclesSinceUpdate = 0;

    public PositionCache(int updateInterval) {
        this.updateInterval = updateInterval;
    }

    public void writePose(DTPosition pos, boolean forceFlush) {
        if (!Double.isNaN(pos.getPose().position.x) && !Double.isNaN(pos.getPose().position.y) && !Double.isNaN(pos.getPose().heading.toDouble())) {
            pos.updateTime();
            String json = gson.toJson(pos);
            editor.putString("CSPosition", json);
            if (forceFlush)
                editor.commit();
            else
                editor.apply();
        }
    }

    public DTPosition readPose() {
        String json = sharedPref.getString("CSPosition", "get failed"); //retrieves the shared preference
        if (json.equals("get failed")) return null;//new CSPosition(new Pose2d(0, 0, 0)); //return a default zeroed TauPos if there's nothing in shared preferences
        return gson.fromJson(json, DTPosition.class); //load the saved JSON into the cached class
    }

    public int update(DTPosition pos, boolean forceUpdate) {
        if (!forceUpdate) {
            if (cyclesSinceUpdate % updateInterval == 0) {
                writePose(pos, false);
            }
        } else {
            writePose(pos, true);
        }
        cyclesSinceUpdate++;
        return cyclesSinceUpdate;
    }


}

