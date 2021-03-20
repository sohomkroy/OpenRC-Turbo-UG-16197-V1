package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class WobbleGoal {
    public static double  servoUpPosition = .3;
    public static double servoDownPosition = .05;//.05;
    public static double servoBackPosition = .8;

    private double servoPosition;

    private final double timeUpToDown = 300;
    private final double timeDownToBack = 300;
    private final double timeBackToUp = 300;
    private CountDownTimer countDownTimer;
    private boolean changed = true;

    public WobbleGoal() {
        countDownTimer = new CountDownTimer();
    }

    public void servoUp() {
        if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.DOWN || StateClass.getWobbleArmState() == StateClass.WobbleArmState.MOVING_DOWN) {
            countDownTimer.setTime(timeUpToDown);
            changed = true;
            StateClass.setWobbleArmState(StateClass.WobbleArmState.MOVING_UP);

        }
        else if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.BACK || StateClass.getWobbleArmState() == StateClass.WobbleArmState.MOVING_BACK) {
            countDownTimer.setTime(timeBackToUp);
            changed = true;
            StateClass.setWobbleArmState(StateClass.WobbleArmState.MOVING_UP);

        }
        else {
            changed = false;
        }
        servoPosition = servoUpPosition;
    }

    public void servoDown() {
        if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.BACK || StateClass.getWobbleArmState() == StateClass.WobbleArmState.MOVING_BACK) {
            countDownTimer.setTime(timeDownToBack);
            changed = true;
            StateClass.setWobbleArmState(StateClass.WobbleArmState.MOVING_DOWN);

        }
        else if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.UP || StateClass.getWobbleArmState() == StateClass.WobbleArmState.MOVING_UP) {
            countDownTimer.setTime(timeUpToDown);
            changed = true;
            StateClass.setWobbleArmState(StateClass.WobbleArmState.MOVING_DOWN);

        }
        else {
            changed = false;
        }
        servoPosition = servoDownPosition;
    }

    public void servoBack() {
        if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.DOWN || StateClass.getWobbleArmState() == StateClass.WobbleArmState.MOVING_DOWN) {
            countDownTimer.setTime(timeDownToBack);
            changed = true;
            StateClass.setWobbleArmState(StateClass.WobbleArmState.MOVING_BACK);
        }
        if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.UP || StateClass.getWobbleArmState() == StateClass.WobbleArmState.MOVING_UP) {
            countDownTimer.setTime(timeBackToUp);
            changed = true;
            StateClass.setWobbleArmState(StateClass.WobbleArmState.MOVING_BACK);

        }
        else {
            changed = false;

        }
        servoPosition = servoBackPosition;
    }

    public double getServoPosition() {
        return servoPosition;
    }

    public void defaultStateReset() {
        servoUp();
        StateClass.setWobbleArmState(StateClass.WobbleArmState.UP);
        servoPosition = 0.6;
//                StateClass.setWobbleArmState(StateClass.WobbleArmState.BACK);
//        servoBack();
    }

    public void defaultStateResetTele() {
        servoBack();
        StateClass.setWobbleArmState(StateClass.WobbleArmState.BACK);
//        servoBack();
    }

    public boolean wasChanged() {
        return changed;
    }

    public void checkServoTimer() {
        if (countDownTimer.timeElapsed()) {
            switch (StateClass.getWobbleArmState()) {
                case MOVING_BACK:
                    StateClass.setWobbleArmState(StateClass.WobbleArmState.BACK);
                    break;
                case MOVING_UP:
                    StateClass.setWobbleArmState(StateClass.WobbleArmState.UP);
                    break;
                case MOVING_DOWN:
                    StateClass.setWobbleArmState(StateClass.WobbleArmState.DOWN);
                    break;
            }
        }
    }

}
