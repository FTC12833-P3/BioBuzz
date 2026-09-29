package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Field Centric Nav", group="Linear OpMode")
public class MM_Field_Centric extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private IMU imu = null;

    double flPower = 0;
    double frPower = 0;
    double blPower = 0;
    double brPower = 0;

    double maxPower;

    private YawPitchRollAngles orientation = null;
    double angle = 0; //TODO get angle
    double strafeAngleError = 0;
    double driveAngleError = 0;
    int currentQuadrant = 1; //can be 1, 2, 3, or 4

    double strafeRatio = 0; //TODO add comment
    double driveRatio = 0; // same as above

    double strafePower = 0; // robot-centric, used to calculate x and y power
    double drivePower = 0; // same as above
    double rotatePower = 0;

    double powerDifference = 0;
    double adjustedPowerDifference = 0;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        initializeIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            orientation = imu.getRobotYawPitchRollAngles();
            angle = orientation.getYaw(AngleUnit.DEGREES);
            getCurrentQuadrant();

            getAngleError();
            calculateVectors();
            calculateDrivePowers();
            setDrivePowers();

            // Check to see if heading reset is requested
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            telemetry.addLine("GYRO TELEMETRY:\n");
            telemetry.addData("Angle", "%.2f Deg. (Heading)\n", orientation.getYaw(AngleUnit.DEGREES));

            telemetry.addLine("DRIVE TELEMETRY:\n");
            telemetry.addData("Front left power", "%4.2f", flPower);
            telemetry.addData("Front right power", "%4.2f", frPower);
            telemetry.addData("Back left power", "%4.2f", blPower);
            telemetry.addData("Back right power", "%4.2f", brPower);
            telemetry.update();
        }
    }

    private void getCurrentQuadrant() {
        if (angle >= 0 && angle < 90) {
            currentQuadrant = 1;
        } else if (angle >= 90 && angle < 180) {
            currentQuadrant = 2;
        } else if (angle >= -180 && angle < -90) {
            currentQuadrant = 3;
        } else if (angle >= -90 && angle < 0) {
            currentQuadrant = 4;
        }
    }

    private void getAngleError() {
        if (currentQuadrant == 1) {
            strafeAngleError = -(90 - angle);
            driveAngleError = angle;
        } else if (currentQuadrant == 2) {
            strafeAngleError = -(90 - angle);
            driveAngleError = 180 - angle;
        } else if (currentQuadrant == 3) {
            strafeAngleError = -90 - angle;
            driveAngleError = -180 - angle;
        } else if (currentQuadrant == 4) {
            strafeAngleError = -90 - angle;
            driveAngleError = angle;
        }
    }

    private void calculateVectors() {
        driveRatio = Math.sin(Math.toRadians(driveAngleError));
        strafeRatio = Math.sin(Math.toRadians(strafeAngleError));

        drivePower = driveRatio / (Math.abs(strafeRatio) + Math.abs(driveRatio));
        strafePower = strafeRatio / (Math.abs(strafeRatio) + Math.abs(driveRatio));
    }

    private void calculateDrivePowers() {
        if (currentQuadrant == 1) {
            powerDifference = drivePower + strafePower;
        } else if (currentQuadrant == 2) {
            powerDifference = drivePower - strafePower;
        } else if (currentQuadrant == 3 || currentQuadrant == 4) {
            powerDifference = strafePower - drivePower;
        }

        adjustedPowerDifference = powerDifference * Math.abs(gamepad1.left_stick_x);

        drivePower = Math.abs(drivePower) + adjustedPowerDifference;
        if (currentQuadrant == 1 || currentQuadrant == 2 || currentQuadrant == 4) { //quadrants 1, 2, or 4
            strafePower = Math.abs(strafePower) + adjustedPowerDifference;
        } else { //quadrant 3
            strafePower = Math.abs(strafePower) - adjustedPowerDifference;
        }

        if (currentQuadrant == 2) {
            strafePower *= -1;
        } else if (currentQuadrant == 4) {
            drivePower *= -1;
        }

        if (gamepad1.left_stick_x < 0) {
            drivePower *= -1;
            strafePower *= -1;
        }

        drivePower *= (Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x));
        strafePower *= (Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x));

        rotatePower = gamepad1.right_stick_x;

        flPower = drivePower + strafePower + rotatePower;
        frPower = drivePower - strafePower - rotatePower;
        blPower = drivePower - strafePower + rotatePower;
        brPower = drivePower + strafePower - rotatePower;

        //normalize
        maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }
    }

    private void setDrivePowers() {
        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    private void initializeIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot)); // if you choose two conflicting directions, this initialization will cause a code exception.
    }
}