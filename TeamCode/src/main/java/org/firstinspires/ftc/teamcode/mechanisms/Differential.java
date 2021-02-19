package org.firstinspires.ftc.teamcode.mechanisms;

public class Differential {
    private double motor1Power;
    private double motor2Power;

    private double intakeSpeed;
    private double turretSpeed;

    public Differential() {
        motor1Power = 0;
        motor2Power = 0;
    }

    public void setIntakeSpeed(double speed) {
        if (Math.abs(speed)+Math.abs(turretSpeed) > 2) {
            //throw error / warning
        }
       else {
            if (Math.abs(speed)+Math.abs(turretSpeed) > 1.8) {

            }
            this.intakeSpeed = speed;
            motor1Power = (intakeSpeed+turretSpeed)/2.0;
            motor2Power = (intakeSpeed-turretSpeed)/2.0;
        }
    }

    public void setTurretSpeed(double speed) {
        if (Math.abs(speed)+Math.abs(intakeSpeed) > 2) {
            //throw error / warning
        }
        else {
            if (Math.abs(speed)+Math.abs(turretSpeed) > 1.8) {

            }
            this.turretSpeed = speed;
            motor1Power = (intakeSpeed+turretSpeed)/2.0;
            motor2Power = (intakeSpeed-turretSpeed)/2.0;
        }
    }

    public double getMotor1Power() {
        return this.motor1Power;
    }

    public double getMotor2Power() {
        return motor2Power;
    }
}
