package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoIntake {
    private final double  servoUpPosition = .6;
    private final double servoDownPosition = .4;
    private final double servoBackPosition = .2;

    private double servoPosition;

    private final double timeUpToDown = 200;
    private final double timeDownToBack = 200;
    private final double timeBackToUp = 200;
    private CountDownTimer countDownTimer = new CountDownTimer();

    public void defaultStateReset() {
        StateClass.setIntakeState(StateClass.IntakeState.STOPPED);
    }

    public ServoIntake() {
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

    private boolean changed;

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
