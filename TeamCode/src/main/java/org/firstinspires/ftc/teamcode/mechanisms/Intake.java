package org.firstinspires.ftc.teamcode.mechanisms;

public class Intake {
    Differential differential;

    private double intakeSpeed = -1;
    private double outtakeSpeed = 1;

    public void defaultStateReset() {
        StateClass.setIntakeState(StateClass.IntakeState.STOPPED);
    }

    public Intake(Differential differential) {
        this.differential = differential;
    }

    public void intakeIn() {
        StateClass.setIntakeState(StateClass.IntakeState.IN);
        differential.setIntakeSpeed(intakeSpeed);
    }

    public void intakeOut() {
        StateClass.setIntakeState(StateClass.IntakeState.OUT);
        differential.setIntakeSpeed(outtakeSpeed);
    }

    public void intakeStop() {
        StateClass.setIntakeState(StateClass.IntakeState.STOPPED);
        differential.setIntakeSpeed(0);
    }
}
