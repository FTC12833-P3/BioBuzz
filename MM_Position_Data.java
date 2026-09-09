package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MM_Position_Data {
    //private final MM_OpMode opMode;
    public GoBildaPinpointDriver odometryController;

    private Pose2D currentPos;
    private MM_Position targetPos = new MM_Position(0,0,0);

    public void setTargetPos(double x, double y, double angle) {
        targetPos.setAll(x, y, angle);

    }
}
