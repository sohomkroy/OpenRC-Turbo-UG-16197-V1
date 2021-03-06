package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CountDownTimer {

    private ElapsedTime timer;

    public CountDownTimer() {
        timer = new ElapsedTime();
    }

    private double time;

    public boolean timeSet = false;

    public boolean timeElapsed() {
        return timer.milliseconds() > time;
    }

    public void setTime(double time) {
        timer.reset();
        this.time = time;
        timeSet = true;
    }


}
