package org.firstinspires.ftc.teamcode.mechanisms;

public class RaisingServo {
    private final double  servoUpPosition = .6;
    private final double servoDownPosition = .4;

    private double servoPosition;

    private final double timeUp = 200;
    private final double timeDown = 200;

    private CountDownTimer countDownTimer;

    public void defaultStateReset() {
        StateClass.setServoRaiserState(StateClass.ServoRaiserState.DOWN);
    }

    public RaisingServo() {
        countDownTimer = new CountDownTimer();
    }

    public void servoUp() {
        if (StateClass.getServoRaiserState() == StateClass.ServoRaiserState.DOWN || StateClass.getServoRaiserState() == StateClass.ServoRaiserState.MOVING_DOWN) {
            countDownTimer.setTime(timeUp);
            changed = true;
        }
        else {
            changed = false;
        }
        StateClass.setServoRaiserState(StateClass.ServoRaiserState.MOVING_UP);
        servoPosition = servoUpPosition;
    }

    public void servoDown() {
        if (StateClass.getServoRaiserState() == StateClass.ServoRaiserState.UP || StateClass.getServoRaiserState() == StateClass.ServoRaiserState.MOVING_UP) {
            countDownTimer.setTime(timeDown);
            changed = true;
        }
        else {
            changed = false;
        }
        StateClass.setServoRaiserState(StateClass.ServoRaiserState.MOVING_DOWN);
        servoPosition = servoDownPosition;
    }

    private boolean changed;

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
