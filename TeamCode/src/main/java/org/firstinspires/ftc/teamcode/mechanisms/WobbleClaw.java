package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class WobbleClaw {
    public static double  servoOpenPosition = .55; //open
    public static double servoClampedPosition = .32; // clamped
    public static double servoBackPosition = .74; // back

    private double servoPosition;

    private final double timeUpToDown = 200;
    private final double timeDownToBack = 200;
    private final double timeBackToUp = 200;
    private CountDownTimer countDownTimer;
    private boolean changed = true;

    public WobbleClaw() {
        countDownTimer = new CountDownTimer();
    }

    public void servoOpen() {
        if (StateClass.getWobbleClawState() == StateClass.WobbleClawState.CLAMPED || StateClass.getWobbleClawState() == StateClass.WobbleClawState.MOVING_CLAMPED) {
            countDownTimer.setTime(timeUpToDown);
            changed = true;
            StateClass.setWobbleClawState(StateClass.WobbleClawState.MOVING_OPEN);

        }
        else if (StateClass.getWobbleClawState() == StateClass.WobbleClawState.BACK || StateClass.getWobbleClawState() == StateClass.WobbleClawState.MOVING_BACK) {
            countDownTimer.setTime(timeBackToUp);
            changed = true;
            StateClass.setWobbleClawState(StateClass.WobbleClawState.MOVING_OPEN);

        }
        else {
            changed = false;
        }
        servoPosition = servoOpenPosition;
    }

    public void servoClamped() {
        if (StateClass.getWobbleClawState() == StateClass.WobbleClawState.BACK || StateClass.getWobbleClawState() == StateClass.WobbleClawState.MOVING_BACK) {
            countDownTimer.setTime(timeDownToBack);
            changed = true;
            StateClass.setWobbleClawState(StateClass.WobbleClawState.MOVING_CLAMPED);

        }
        else if (StateClass.getWobbleClawState() == StateClass.WobbleClawState.OPEN || StateClass.getWobbleClawState() == StateClass.WobbleClawState.MOVING_OPEN) {
            countDownTimer.setTime(timeUpToDown);
            changed = true;
            StateClass.setWobbleClawState(StateClass.WobbleClawState.MOVING_CLAMPED);

        }
        else {
            changed = false;
        }
        servoPosition = servoClampedPosition;
    }

    public void servoBack() {
        if (StateClass.getWobbleClawState() == StateClass.WobbleClawState.CLAMPED || StateClass.getWobbleClawState() == StateClass.WobbleClawState.MOVING_CLAMPED) {
            countDownTimer.setTime(timeDownToBack);
            changed = true;
            StateClass.setWobbleClawState(StateClass.WobbleClawState.MOVING_BACK);
        }
        if (StateClass.getWobbleClawState() == StateClass.WobbleClawState.OPEN || StateClass.getWobbleClawState() == StateClass.WobbleClawState.MOVING_OPEN) {
            countDownTimer.setTime(timeBackToUp);
            changed = true;
            StateClass.setWobbleClawState(StateClass.WobbleClawState.MOVING_BACK);

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

        servoClamped();
        StateClass.setWobbleClawState(StateClass.WobbleClawState.CLAMPED);
    }

    public boolean wasChanged() {
        return changed;
    }

    public void checkServoTimer() {
        if (countDownTimer.timeElapsed()) {
            switch (StateClass.getWobbleClawState()) {
                case MOVING_BACK:
                    StateClass.setWobbleClawState(StateClass.WobbleClawState.BACK);
                    break;
                case MOVING_OPEN:
                    StateClass.setWobbleClawState(StateClass.WobbleClawState.OPEN);
                    break;
                case MOVING_CLAMPED:
                    StateClass.setWobbleClawState(StateClass.WobbleClawState.CLAMPED);
                    break;
            }
        }
    }

}
