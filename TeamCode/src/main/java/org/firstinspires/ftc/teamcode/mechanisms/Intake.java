package org.firstinspires.ftc.teamcode.mechanisms;

public class Intake {
    StateClass stateClass;
    Differential differential;

    private double intakeSpeed = -.4;
    private double outtakeSpeed = .4;

    public Intake(StateClass stateClass, Differential differential) {
        this.stateClass = stateClass;
        this.stateClass.setIntakeState(StateClass.IntakeState.STOPPED);
        this.differential = differential;
    }

    public void intakeIn() {
        this.stateClass.setIntakeState(StateClass.IntakeState.IN);
        differential.setIntakeSpeed(intakeSpeed);
    }

    public void intakeOut() {
        this.stateClass.setIntakeState(StateClass.IntakeState.OUT);
        differential.setIntakeSpeed(outtakeSpeed);
    }

    public void intakeStop() {
        this.stateClass.setIntakeState(StateClass.IntakeState.STOPPED);
        differential.setIntakeSpeed(0);
    }
}
