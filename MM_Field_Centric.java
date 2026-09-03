package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Field Centric Nav", group="Linear OpMode")
public class MM_Field_Centric extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double maxPower;

            double yPower = -gamepad1.left_stick_y;  // pushing stick forward gives negative value
            double xPower = gamepad1.left_stick_x;
            double rotatePower = gamepad1.right_stick_x;

            double flPower = yPower + xPower + rotatePower; //TODO probably should change for field-centric
            double frPower = yPower - xPower - rotatePower;
            double blPower = yPower - xPower + rotatePower;
            double brPower = yPower + xPower - rotatePower;

            // Normalize
            maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

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

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", flPower, frPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", blPower, brPower);
            telemetry.update();
        }
    }}
