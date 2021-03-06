package org.firstinspires.ftc.teamcode.mechanisms;

public class ServoIntake {
    private StateClass stateClass;

    private double servoUpPosition = .6;
    private double servoDownPosition = .4;
    private double servoBackPosition = .2;

    private double servoPosition;

    public ServoIntake(StateClass stateClass) {
        this.stateClass = stateClass;
    }

    public void servoUp() {
        if (stateClass.getServoIntakeState() != StateClass.ServoIntakeState.UP) {
            changed = true;
        }
        else {
            changed = false;
        }
        stateClass.setServoIntakeState(StateClass.ServoIntakeState.UP);
        servoPosition = servoUpPosition;
    }

    public void servoDown() {
        if (stateClass.getServoIntakeState() != StateClass.ServoIntakeState.DOWN) {
            changed = true;
        }
        else {
            changed = false;
        }
        stateClass.setServoIntakeState(StateClass.ServoIntakeState.DOWN);
        servoPosition = servoDownPosition;
    }

    public void servoBack() {
        if (stateClass.getServoIntakeState() != StateClass.ServoIntakeState.BACK) {
            changed = true;
        }
        else {
            changed = false;
        }
        stateClass.setServoIntakeState(StateClass.ServoIntakeState.BACK);
        servoPosition = servoBackPosition;
    }

    public double getServoPosition() {
        return servoPosition;
    }

    private boolean changed;

    public boolean wasChanged() {
        return changed;
    }


}
