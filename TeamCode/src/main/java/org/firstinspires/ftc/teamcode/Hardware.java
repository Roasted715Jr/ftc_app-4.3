package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Hardware<T extends GenericOpMode> {
    private static final double INCH_PER_MM = 0.03937007874;
    private static final double WHEEL_DIAMETER_INCH = 100.965 * INCH_PER_MM;
    private static final double WHEEL_CIRCUMFERENCE_INCH = WHEEL_DIAMETER_INCH * Math.PI;
    private static final int REV_CORE_HEX_COUNTS_PER_REVOLUTION = 288;
    private static final int NEVEREST_40_COUNTS_PER_REVOLUTION = 1120;
    //    private static final int NEVEREST_20_COUNTS_PER_REVOLUTION = 537; //Is actually 537.6, but setting the motors requires an int so it will truncate to 537 anyways
    private static final double TURN_SPEED = 0.25;

    private static final int COMP_BOT = 0;
    private static final int MATT_TINY_BOT = 1;
    private static final int TINY_BOT = 2;
    private static final int robotType = COMP_BOT;

    static final String VUFORIA_LICENSE_KEY = "Abq1tHr/////AAABmYC8ioniS0f2gyQRx7fZlTWMwyYcrV/bnslJvcDe0AhxA/GAkYTIdNbPWjYtplipzvASUZRGR+AoGDI1dKyuCFCc4qy1eVbx8NO4nuAKzeGoncY7acvfol19suW5Zl29E+APEV0CG4GVBe4R+bZ/Xyd2E7CZ7AcrLbWM8+SJiMCDnxJa3J0ozBHMPMs6GNFyYS6YCVNMkFcLEKxDicwXqpuJddG5XenbAs8ot9UT11WRYZjpprLkSRtM1/OyigcUeb0wk2PL6lFVBMHMZbWK5HkJEmBoN5+v2fP6zouj0GPGyEh/eV8Xe71LhBz0WXKd180hUCowZVBfdsTtuYwFiBkAyRLtiQQb4/b80sAx1b6s";

    private static int PINION_TEETH;
    private static int SPUR_TEETH;

    private BNO055IMU imu;
    ColorSensor rightSensor, leftSensor;
    private HardwareMap hardwareMap;
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor liftMotor;
    private Servo baseServo;
    private CRServo tapeServo, tapeControl;
    private T runningOpMode;
    private Thread lifterBtnListener;
    private TouchSensor lifterBtn;

    private WebcamName webcamName;

    Hardware (T runningOpMode) {
        this.runningOpMode = runningOpMode;
    }

    void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        switch (robotType) {
            case MATT_TINY_BOT:
                rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
                leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
                leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled = true;
                parameters.loggingTag = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);

                PINION_TEETH = 1;
                SPUR_TEETH = 1;
                break;
            case COMP_BOT:
                rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
                leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
                rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightSensor = hardwareMap.get(ColorSensor.class, "rightSensor");
                leftSensor = hardwareMap.get(ColorSensor.class, "leftSensor");
                baseServo = hardwareMap.get(Servo.class, "baseServo");
                tapeServo = hardwareMap.get(CRServo.class, "tapeServo");
                tapeControl = hardwareMap.get(CRServo.class, "tapeControl");

                //Parameters will already be defined in the scope... the scope spans between all the cases, but you can't do anything outside a case :/
                parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled = true;
                parameters.loggingTag = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);

                PINION_TEETH = 35;
                SPUR_TEETH = 45;
                break;
            case TINY_BOT:
                rightMotor = hardwareMap.get(DcMotor.class, "driveR");
                leftMotor = hardwareMap.get(DcMotor.class, "driveL");

                PINION_TEETH = 1;
                SPUR_TEETH = 1;
                break;
        }

        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorPowers(0, 0);

        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        lifterBtnListener = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                if (lifterBtn.isPressed())
//                    liftReset();
//            }
//        });

//        lifterBtnListener.start();
    }

    void initCamera() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

//    private boolean isBetween(int[] val, int[] min, int[] max) {
//        boolean isBetween = true;
//
//        for (int i = 0; i < val.length; i++) {
//            if (!(min[i] <= val[i] && val[i] <= max[i])) {
//                isBetween = false;
//                break;
//            }
//        }
//
//        return isBetween;
//    }

    double getWheelCircumferenceInch() {
        return WHEEL_CIRCUMFERENCE_INCH;
    }

    Servo getBaseServo() {
        return baseServo;
    }

    CRServo getTapeServo() {
        return tapeServo;
    }

    CRServo getTapeControl() {
        return tapeControl;
    }

    WebcamName getWebcam() {
        return webcamName;
    }

    double getYaw() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double mmToInch(double mm) {
        return mm * INCH_PER_MM;
    }

    void goDistance(double inchToTravel, double speed) {
        double revolutions = inchToTravel / (WHEEL_CIRCUMFERENCE_INCH * PINION_TEETH / SPUR_TEETH);

        //Rev Core Hex Motors:
        //  Motor: 4 counts/revolution
        //  Output: 288 counts/revolution
        //Neverest 40:
        //  Motor: 7 counts/revolution
        //  Output: 1120 counts/revolution

        if (robotType == MATT_TINY_BOT || robotType == TINY_BOT)
                goEncoderCounts((int) (revolutions * REV_CORE_HEX_COUNTS_PER_REVOLUTION), speed);
        else if (robotType == COMP_BOT)
                goEncoderCounts((int) (revolutions * NEVEREST_40_COUNTS_PER_REVOLUTION), speed);

        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void goEncoderCounts(int counts, double speed) {
        setMotorTargetPositions(counts);
        setMotorPowers(speed);

        while ((rightMotor.isBusy() || leftMotor.isBusy())) {}

        setMotorPowers(0);
    }

    void liftExtendFull() {
//        liftToPos(8500, elapsedTime);
        liftToPos(8650); //8300, 8500
    }

    void liftExtendPartial() {
        liftToPos(7500);
    }

    void liftReset() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Lift some more to ensure the button is no longer pressed?
    }

    void liftRetract() {
        liftToPos(100);
    }

    void liftToPos(int counts) {
        //The value is negative for some odd reason
        //7000 to bottom of hook
        //8500 to top of hook

        setLiftRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setTargetPosition(-counts);
        setLiftPower(1);

        while (liftMotor.isBusy()) {
            runningOpMode.addTelemetry("liftMotor Position: " + -(liftMotor.getCurrentPosition()) + ", aiming for " + counts);
            runningOpMode.updateTelemetry();
        }

        setLiftPower(0);
    }

    void liftToPos(int counts, ElapsedTime elapsedTime) {
        setLiftRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(-counts);
        setLiftPower(1);

        while (liftMotor.isBusy() && elapsedTime.seconds() < 3) {}

        setLiftPower(0);
    }

    void moveServo(double pos) {
        baseServo.setPosition(pos);
    }

    void setLiftPower(double liftSpeed) {
        liftMotor.setPower(-liftSpeed);
    }

    void setMotorPowers(double motorSpeeds) {
        rightMotor.setPower(motorSpeeds);
        leftMotor.setPower(motorSpeeds);
    }

    void setMotorPowers(double rightSpeed, double leftSpeed) {
        rightMotor.setPower(rightSpeed);
        leftMotor.setPower(leftSpeed);
    }

    private void setMotorRunModes(DcMotor.RunMode runMode) {
        //RUN_WITHOUT_ENCODER: Set power straight to motor, but still track encoders
        //RUN_USING_ENCODER: Makes sure motors are running at the right speed
        //RUN_TO_POSITION: Motor will go to and hold position that is set
        //RESET_ENCODER: Resets encoder values to 0

        rightMotor.setMode(runMode);
        leftMotor.setMode(runMode);
    }

    void setLiftRunMode(DcMotor.RunMode runMode) {
        liftMotor.setMode(runMode);
    }

    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        rightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private void setMotorTargetPositions(int pos) {
        //If we are changing the target we should also reset the encoders
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setTargetPosition(pos);
        leftMotor.setTargetPosition(pos);
        setMotorRunModes(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void stop() {
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorPowers(0);
        lifterBtnListener.stop();
    }

    void turnDegrees(double degreesToTurn) {
        double startYaw = getYaw();
        double target = startYaw + degreesToTurn;

//        Log.i(TAG, "Target: " + target);
//        Log.i(TAG, "Start Yaw: " + startYaw);
//        Log.i(TAG, "Degreees to Turn: " + degreesToTurn);

        if (degreesToTurn < 0) { //Turn right
            while (getYaw() > target)
                setMotorPowers(-TURN_SPEED, TURN_SPEED);
            setMotorPowers(0);
        } else { //Turn left
            while (getYaw() < target)
                setMotorPowers(TURN_SPEED, -TURN_SPEED);
            setMotorPowers(0);
        }
    }

    void turnToDegree(double target) {
        double startYaw = getYaw();

        if (startYaw > target) { //Turn right
            while (getYaw() > target)
                setMotorPowers(-TURN_SPEED, TURN_SPEED);
            setMotorPowers(0);
        } else { //Turn left
            while (getYaw() < target)
                setMotorPowers(TURN_SPEED, -TURN_SPEED);
            setMotorPowers(0);
        }
    }

    void waitForTape() {
        setMotorPowers(0.75);

        int redR, redL, blueR, blueL;
        double rightValue = 1, leftValue = 1;
        float[] hsvValuesR = new float[3], hsvValuesL = new float[3];
        float hueR, hueL;

        while (rightValue > 0 || leftValue > 0) {
            redR = rightSensor.red();
            blueR = rightSensor.blue();
            redL = leftSensor.red();
            blueL = leftSensor.blue();

            Color.RGBToHSV(rightSensor.red(),
                    rightSensor.green(),
                    rightSensor.blue(),
                    hsvValuesR);
            Color.RGBToHSV(leftSensor.red(),
                    leftSensor.green(),
                    leftSensor.blue(),
                    hsvValuesL);

            hueR = hsvValuesR[0];
            hueL = hsvValuesL[0];

            if ((redR > 1000 && (hueR < 10 || hueR > 350)) || (blueR > 1000 && (hueR < 230 && hueR > 190)))
                rightValue = 0;

            if ((redL > 1000 && (hueL < 10 || hueL > 350)) || (blueL > 1000 && (hueL < 230 && hueL > 190)))
                leftValue = 0;

            setMotorPowers(rightValue, leftValue);

            runningOpMode.addTelemetry("hueR: " + hueR + "\nhueL: " + hueL);
        }

        setMotorPowers(0);
    }
}
