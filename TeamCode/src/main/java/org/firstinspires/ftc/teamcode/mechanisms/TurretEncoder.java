package org.firstinspires.ftc.teamcode.mechanisms;

public class TurretEncoder {
    private double turretAngle;
    private double initialAngle;
    private double initialTicks;
    public TurretEncoder() {

    }

    public void setTurretAngle(double encoderTicks) {
        this.turretAngle = initialAngle + (encoderTicks-initialTicks)/8192*28/119*360;
    }
    public void setInitialAngle(double initialAngle) {
        this.initialAngle = initialAngle;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public void setInitialTicks(double initialTicks) {
        this.initialTicks = initialTicks;
    }
}
