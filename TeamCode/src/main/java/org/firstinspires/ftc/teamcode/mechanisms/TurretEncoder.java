package org.firstinspires.ftc.teamcode.mechanisms;

public class TurretEncoder {
    private double turretAngle;
    private double initialAngle;

    public TurretEncoder() {

    }

    public void setTurretAngle(double encoderTicks) {
        this.turretAngle = initialAngle + encoderTicks/8192*360;
    }
    public void setInitialAngle(double initialAngle) {
        this.initialAngle = initialAngle;
    }

    public double getTurretAngle() {
        return turretAngle;
    }
}
