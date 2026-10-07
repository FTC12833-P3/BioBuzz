package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field Centric Nav", group="MM")
public class MM_Field_Centric extends LinearOpMode {
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;
    private IMU imu = null;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        initializeIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveWithSticks(); //field-centric navigation

            telemetry.update();
        }
    }

    private void driveWithSticks() {
        double heading = getHeading();
        double sinHeading = Math.sin(Math.toRadians(heading));
        double cosHeading = Math.cos(Math.toRadians(heading));

        double drivePower = (sinHeading * -gamepad1.left_stick_y) + (cosHeading * gamepad1.left_stick_x);
        double strafePower = (-cosHeading * -gamepad1.left_stick_y) + (sinHeading * gamepad1.left_stick_x);
        double rotatePower = gamepad1.right_stick_x;

        double flPower = drivePower + strafePower + rotatePower;
        double frPower = drivePower - strafePower - rotatePower;
        double blPower = drivePower - strafePower + rotatePower;
        double brPower = drivePower + strafePower - rotatePower;

        //normalize for the next 8 lines
        double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }

        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);

        telemetry.addLine("DRIVE TELEMETRY:\n"); //TODO remove telemetry once finished
        telemetry.addData("Front left power", "%4.2f", flPower);
        telemetry.addData("Front right power", "%4.2f", frPower);
        telemetry.addData("Back left power", "%4.2f", blPower);
        telemetry.addData("Back right power", "%4.2f", brPower);
    }

    private double getHeading() {
        // Check to see if heading reset is requested
        if (gamepad1.yWasPressed()) { //TODO consider moving heading stuff to new method
            imu.resetYaw();
        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addLine("GYRO TELEMETRY:\n");
        telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
        telemetry.addData("Heading", "%.2f Deg. (Heading)\n", heading);
        return heading;
    }

    private void initializeIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot)); // if you choose two conflicting directions, this initialization will cause a code exception.
    }
}