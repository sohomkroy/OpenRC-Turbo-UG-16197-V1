package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ServoIntake {
    public static double  servoUpPosition = .2;
    public static double servoDownPosition = .4;
    public static double servoBackPosition = .2;

    private double servoPosition;

    private final double timeUpToDown = 200;
    private final double timeDownToBack = 200;
    private final double timeBackToUp = 200;
    private CountDownTimer countDownTimer;
    private boolean changed = true;

    public ServoIntake() {
        countDownTimer = new CountDownTimer();
    }

    public void servoUp() {
        if (StateClass.getServoIntakeState() == StateClass.ServoIntakeState.DOWN || StateClass.getServoIntakeState() == StateClass.ServoIntakeState.MOVING_DOWN) {
            countDownTimer.setTime(timeUpToDown);
            changed = true;
        }
        else if (StateClass.getServoIntakeState() == StateClass.ServoIntakeState.BACK || StateClass.getServoIntakeState() == StateClass.ServoIntakeState.MOVING_BACK) {
            countDownTimer.setTime(timeBackToUp);
            changed = true;
        }
        else {
            changed = false;
        }
        StateClass.setServoIntakeState(StateClass.ServoIntakeState.MOVING_UP);
        servoPosition = servoUpPosition;
    }

    public void servoDown() {
        if (StateClass.getServoIntakeState() == StateClass.ServoIntakeState.BACK || StateClass.getServoIntakeState() == StateClass.ServoIntakeState.MOVING_BACK) {
            countDownTimer.setTime(timeDownToBack);
            changed = true;
        }
        else if (StateClass.getServoIntakeState() == StateClass.ServoIntakeState.UP || StateClass.getServoIntakeState() == StateClass.ServoIntakeState.MOVING_UP) {
            countDownTimer.setTime(timeUpToDown);
            changed = true;
        }
        else {
            changed = false;
        }
        StateClass.setServoIntakeState(StateClass.ServoIntakeState.MOVING_DOWN);
        servoPosition = servoDownPosition;
    }

    public void servoBack() {
        if (StateClass.getServoIntakeState() == StateClass.ServoIntakeState.DOWN || StateClass.getServoIntakeState() == StateClass.ServoIntakeState.MOVING_DOWN) {
            countDownTimer.setTime(timeDownToBack);
            changed = true;
        }
        if (StateClass.getServoIntakeState() == StateClass.ServoIntakeState.UP || StateClass.getServoIntakeState() == StateClass.ServoIntakeState.MOVING_UP) {
            countDownTimer.setTime(timeBackToUp);
            changed = true;
        }
        else {
            changed = false;
        }
        StateClass.setServoIntakeState(StateClass.ServoIntakeState.MOVING_BACK);
        servoPosition = servoBackPosition;
    }

    public double getServoPosition() {
        return servoPosition;
    }

    public void defaultStateReset() {
        StateClass.setServoIntakeState(StateClass.ServoIntakeState.UP);
        servoUp();
    }

    public boolean wasChanged() {
        return changed;
    }

    public void checkServoTimer() {
        if (countDownTimer.timeElapsed()) {
            switch (StateClass.getServoIntakeState()) {
                case MOVING_BACK:
                    StateClass.setServoIntakeState(StateClass.ServoIntakeState.BACK);
                    break;
                case MOVING_UP:
                    StateClass.setServoIntakeState(StateClass.ServoIntakeState.UP);
                    break;
                case MOVING_DOWN:
                    StateClass.setServoIntakeState(StateClass.ServoIntakeState.DOWN);
                    break;
            }
        }
    }

}
