package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

abstract class GenericOpMode extends LinearOpMode {
    GenericOpMode() {}

    void addTelemetry(String msg) {
        telemetry.addData("Autonomous", msg);
    }

    void updateTelemetry() {
        telemetry.update();
    }

    @Override
    public void waitForStart() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
    }
}
