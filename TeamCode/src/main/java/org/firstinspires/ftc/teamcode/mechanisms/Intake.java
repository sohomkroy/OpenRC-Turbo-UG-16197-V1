package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Intake {
    Differential differential;

    public static double intakeSpeed = 1;
    public static double intakeFastSpeed = 1;
    public static double outtakeSpeed = -1;
    public static double outtakeFastSpeed = -1;
    public static double startingIntakeTime = 200;
    CountDownTimer timer = new CountDownTimer();

    public void defaultStateReset() {
        StateClass.setIntakeState(StateClass.IntakeState.STOPPED);
    }

    public Intake(Differential differential) {
        this.differential = differential;
    }

    public void intakeIn() {
        StateClass.setIntakeState(StateClass.IntakeState.IN);
        differential.setIntakeSpeed(intakeFastSpeed);
        timer.setTime(startingIntakeTime);
    }

    public void updateTimer() {
        if (StateClass.getIntakeState() == StateClass.IntakeState.IN && timer.timeElapsed()) {
            differential.setIntakeSpeed(intakeSpeed);
        }
        if (StateClass.getIntakeState() == StateClass.IntakeState.OUT && timer.timeElapsed()) {
            differential.setIntakeSpeed(outtakeSpeed);
        }
    }

    public void intakeOut() {
        StateClass.setIntakeState(StateClass.IntakeState.OUT);
        differential.setIntakeSpeed(outtakeFastSpeed);
        timer.setTime(startingIntakeTime);
    }

    public void intakeStop() {
        StateClass.setIntakeState(StateClass.IntakeState.STOPPED);
        differential.setIntakeSpeed(0);
    }
}
