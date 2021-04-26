package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class StickServo {
    public static double  servoUpPosition = 0;
    public static double servoDownPosition = 0.51;
    public static double timeUp = 50;
    public static double timeDown = 50;
    private double servoPosition;
    private CountDownTimer countDownTimer;
    private boolean changed = true;

    public StickServo() {
        countDownTimer = new CountDownTimer();
    }

    public void defaultStateReset() {
        StateClass.setStickState(StateClass.StickState.DOWN);
    }

    public void servoUp() {
        if (StateClass.getStickState() == StateClass.StickState.DOWN || StateClass.getStickState() == StateClass.StickState.MOVING_DOWN) {
            countDownTimer.setTime(timeUp);
            changed = true;
            StateClass.setStickState(StateClass.StickState.MOVING_UP);
        }
        else {
            changed = false;
        }


        servoPosition = servoUpPosition;
    }

    public void servoDown() {
        if (StateClass.getStickState() == StateClass.StickState.UP || StateClass.getStickState() == StateClass.StickState.MOVING_UP) {
            countDownTimer.setTime(timeDown);
            changed = true;
            StateClass.setStickState(StateClass.StickState.MOVING_DOWN);
        }
        else {
            changed = false;
        }
        servoPosition = servoDownPosition;
    }

    public boolean wasChanged() {
        return changed;
    }

    public void checkServoTimer() {
        if (countDownTimer.timeElapsed()) {
            switch (StateClass.getStickState()) {
                case MOVING_UP:
                    StateClass.setStickState(StateClass.StickState.UP);
                    break;
                case MOVING_DOWN:
                    StateClass.setStickState(StateClass.StickState.DOWN);
                    break;
            }
        }
    }

    public double getServoPosition() {
        return servoPosition;
    }
}
