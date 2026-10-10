package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MM_DrivePowers {
    private final MM_OpMode opMode;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;

    public MM_DrivePowers(MM_OpMode opMode) {
        this.opMode = opMode;
    }

    public void updateDrivePowers(double flPower, double frPower, double blPower, double brPower) {
        this.flPower = flPower;
        this.frPower = frPower;
        this.blPower = blPower;
        this.brPower = brPower;
    } // TODO figure out if this should include normalize and if it should be named setDrivePowers or updateDrivePowers

    public void setDrivePowers(double forwardPower, double strafePower, double rotatePower) {
        flPower = forwardPower + strafePower + rotatePower;
        frPower = forwardPower - strafePower - rotatePower;
        blPower = forwardPower - strafePower + rotatePower;
        brPower = forwardPower + strafePower - rotatePower;

        normalize();
    }

    private void normalize() {
        double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }
    }

    public void drivePowerTelemetry() {
        opMode.telemetry.addLine("DRIVE TELEMETRY:\n"); //TODO remove telemetry once finished
        opMode.telemetry.addData("Front left power", "%4.2f", flPower);
        opMode.telemetry.addData("Front right power", "%4.2f", frPower);
        opMode.telemetry.addData("Back left power", "%4.2f", blPower);
        opMode.telemetry.addData("Back right power", "%4.2f", brPower);
    }

    public double getFlPower() {
        return flPower;
    }

    public double getFrPower() {
        return frPower;
    }

    public double getBlPower() {
        return blPower;
    }

    public double getBrPower() {
        return brPower;
    }
}
