package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RaisingServo {
    public static double  servoUpPosition = .41;
    public static double servoDownPosition = .68;

    private double servoPosition;

    public static double timeUp = 800;
    public static double timeDown = 350;

    private CountDownTimer countDownTimer;

    public void defaultStateReset() {
        StateClass.setServoRaiserState(StateClass.ServoRaiserState.DOWN);
    }

    public RaisingServo() {
        countDownTimer = new CountDownTimer();
    }
    private boolean changed = true;

    public void servoUp() {
        if (StateClass.getServoRaiserState() == StateClass.ServoRaiserState.DOWN || StateClass.getServoRaiserState() == StateClass.ServoRaiserState.MOVING_DOWN) {
            countDownTimer.setTime(timeUp);
            changed = true;
            StateClass.setServoRaiserState(StateClass.ServoRaiserState.MOVING_UP);
        }
        else {
            changed = false;
        }


        servoPosition = servoUpPosition;
    }

    public void servoDown() {
        if (StateClass.getServoRaiserState() == StateClass.ServoRaiserState.UP || StateClass.getServoRaiserState() == StateClass.ServoRaiserState.MOVING_UP) {
            countDownTimer.setTime(timeDown);
            changed = true;
            StateClass.setServoRaiserState(StateClass.ServoRaiserState.MOVING_DOWN);
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
            switch (StateClass.getServoRaiserState()) {
                case MOVING_UP:
                    StateClass.setServoRaiserState(StateClass.ServoRaiserState.UP);
                    break;
                case MOVING_DOWN:
                    StateClass.setServoRaiserState(StateClass.ServoRaiserState.DOWN);
                    break;
            }
        }
    }

    public double getServoPosition() {
        return servoPosition;
    }
}
