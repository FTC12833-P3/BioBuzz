package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field Centric Nav", group="MM")
public class MM_FieldCentric extends LinearOpMode {
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;
    private IMU imu = null;
    private MM_DrivePowers drivePowers = null;

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

        drivePowers = new MM_DrivePowers(this);
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

        double forwardPower = (sinHeading * -gamepad1.left_stick_y) + (cosHeading * gamepad1.left_stick_x);
        double strafePower = (-cosHeading * -gamepad1.left_stick_y) + (sinHeading * gamepad1.left_stick_x);
        double rotatePower = gamepad1.right_stick_x;

        drivePowers.setDrivePowers(forwardPower, strafePower, rotatePower);

        frontLeftDrive.setPower(drivePowers.getFlPower());
        frontRightDrive.setPower(drivePowers.getFrPower());
        backLeftDrive.setPower(drivePowers.getBlPower());
        backRightDrive.setPower(drivePowers.getBrPower());

        drivePowers.drivePowerTelemetry(); //comment out as needed for testing
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