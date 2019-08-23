package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class GenericOpMode extends LinearOpMode {
    void addTelemetry(String msg) {
        telemetry.addData("Autonomous", msg);
    }

    void updateTelemetry() {
        telemetry.update();
    }

    public void waitForStart() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
    }

    public abstract void runOpMode() throws InterruptedException;
}
