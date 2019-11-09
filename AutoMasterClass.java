package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
public abstract class AutoMasterClass extends LinearOpMode {
    private ElapsedTime runtime;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, slide;
    private DcMotorEx[] motors;
    private Servo servoright,servoleft,platformmover1,platformmover2;
    private BNO055IMU imu;
    private PID Pid = new PID(.022, 0.0003, 0.0022);
    private PID strafePID = new PID(.02, .0003, .002);
    private Robot_Navigation kb = new Robot_Navigation();
    private vuforia ab = new vuforia();
    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;
    private int position;
    private final double GEAR_RATIO = 1.00000, WHEEL_DIAMETER = 4, WHEEL_TICKS_PER_REV = 560;
    private final double C = 537 / (Math.PI * 4 * (5.0 / 6)), STRAFE_COEFFICIENT = 1.12943302;

    public enum SKYSTONE_POSITION {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public enum ALLIANCE_COLOR {
        RED,
        BLUE
    }
    public void initVuforia() {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        ab.initVuforia(parameters);
    }

    public void move(double targetHeading, double power, double ticks) {

        ticks = ticks * 537 / (Math.PI * 4 * (5.0 / 6));
        int realticks = 0;
        realticks = (int) ticks;
        setTargetPosition(realticks);
        resetMotors();
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            correction(.3,0,"stright",false);
        }

        halt();
    }
    public void moveByTime(double time, double targetHeading, double power) {
        double current = runtime.time();

        while (runtime.time() - current < time) {
            correction(power, targetHeading, "straight", false);
        }
        halt();
    }

    public void moveSlideByTicks() {
        double basePosition = slide.getCurrentPosition();

        while (Math.abs(basePosition - slide.getCurrentPosition()) < 300) {
            slide.setPower(-.2);
        }
        slide.setPower(0);
    }
    public void setTargetPosition(int targetPosition) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(targetPosition);
        }
    }
    public void initialize() {


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRearDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRearDrive");
        slide = hardwareMap.get(DcMotorEx.class, "lift");
        servoright = hardwareMap.servo.get("servoright");
        servoleft = hardwareMap.servo.get("servoleft");
        platformmover1 = hardwareMap.servo.get("lift1");
        platformmover2 = hardwareMap.servo.get("lift2");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        platformmover1.setDirection(Servo.Direction.REVERSE);
        platformmover2.setDirection(Servo.Direction.FORWARD);
        servoleft.setDirection(Servo.Direction.FORWARD);
        servoright.setDirection(Servo.Direction.REVERSE);
        motors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack, slide};

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        runtime = new ElapsedTime();
    }
    public void gripPlatform() {
        platformmover1.setPosition(.7);
        platformmover2.setPosition(.7);
    }

    public void releasePlatform() {
        platformmover1.setPosition(1);
        platformmover2.setPosition(1);
    }

    public void heartbeat() throws InterruptedException{
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if(!opModeIsActive()) {
            throw new InterruptedException();
        }
    }

    public SKYSTONE_POSITION determineSkystonePlacement(ALLIANCE_COLOR color) throws InterruptedException {
        resetMotors();

        double baseSlidePosition = slide.getCurrentPosition();
        double yPosition = 0;
        double inches = 16;

        ab.targetsSkyStone.activate();
        pause(0.5);

        while (!ab.targetVisible && motorsBusy((int)(inches*C*STRAFE_COEFFICIENT))) {
            heartbeat();
            correction(.125, 0, "straferight", false);
            if (Math.abs(baseSlidePosition - slide.getCurrentPosition()) < 300) {
                slide.setPower(-.2);
            }
            else
                slide.setPower(0);
            yPosition = ab.getYPosition();


            telemetry.update();
        }
        halt();

        double current = runtime.time();
        while (Math.abs(current - runtime.time()) < 1) {
            yPosition = ab.getYPosition();
            telemetry.addData("yPosition", yPosition);


            telemetry.update();
        }
        ab.targetsSkyStone.deactivate();

        if (color.equals(ALLIANCE_COLOR.RED)) {
            return determineRedPosition(motorsBusy((int)(inches*C*STRAFE_COEFFICIENT)), yPosition);
        }
        return determineBluePosition(yPosition);
    }
    public void pause ( double time) throws InterruptedException {
        double pause = runtime.time();
        while (runtime.time() - pause < time) {
            heartbeat();
            telemetry.addData("Paused ", time);
            telemetry.update();
        }

    }
    public void halt() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }
    public SKYSTONE_POSITION determineRedPosition(boolean scanned, double yPosition) {
        if (yPosition > 1)
            return SKYSTONE_POSITION.RIGHT;
        else if ((!scanned && yPosition == 0) || yPosition < -8)
            return SKYSTONE_POSITION.LEFT;
        return SKYSTONE_POSITION.MIDDLE;
    }

    public SKYSTONE_POSITION determineBluePosition(double yPosition) {
        if (yPosition < -2) {
            return SKYSTONE_POSITION.MIDDLE;
        }
        else if (yPosition > 2) {
            return SKYSTONE_POSITION.RIGHT;
        }
        return SKYSTONE_POSITION.LEFT;
    }
    public void resetMotors() {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public boolean motorsBusy(int ticks) {
        return Math.abs(leftBack.getCurrentPosition()) < ticks && Math.abs(rightBack.getCurrentPosition()) < ticks && Math.abs(leftFront.getCurrentPosition()) < ticks && Math.abs(rightFront.getCurrentPosition()) < ticks;
    }
    public double currentAngle() {
        //returns heading from gyro using unit circle values (-180 to 180 degrees, -pi to pi radians. We're using degrees)
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public void correction(double power, double targetHeading, String movementType, boolean inverted)
    {
        //sets target and current angles
        double target = targetHeading;
        double current = currentAngle();


        //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
        if (targetHeading < -135 && currentAngle() > 135) {
            target = targetHeading + 360.0;
        }
        else if (targetHeading > 135 && currentAngle() < -135) {
            current = currentAngle() + 360.0;
        }

        if (target > 180) {
            target-=360;
        }
        else if (target < -180) {
            target += 360;
        }

        //PD correction for both regular and spline motion
        if (movementType.contains("straight") || movementType.contains("spline")) {
            leftFront.setPower(Range.clip(power + Pid.getCorrection(current - target, runtime), -1.0, 1.0));
            rightFront.setPower(Range.clip(power - Pid.getCorrection(current - target, runtime), -1.0, 1.0));
            leftBack.setPower(Range.clip(power + Pid.getCorrection(current - target, runtime), -1.0, 1.0));
            rightBack.setPower(Range.clip(power - Pid.getCorrection(current - target, runtime), -1.0, 1.0));
        }

        //pd correction for strafe motion. Right and left are opposites
        else if (movementType.contains("strafe")) {
            if (movementType.contains("left")) {
                leftFront.setPower(Range.clip(-power + strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
                rightFront.setPower(Range.clip(power - strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
                leftBack.setPower(Range.clip(power + strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
                rightBack.setPower(Range.clip(-power - strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
            }
            else if (movementType.contains("right")) {
                leftFront.setPower(Range.clip(power + strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
                rightFront.setPower(Range.clip(-power - strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
                leftBack.setPower(Range.clip(-power + strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
                rightBack.setPower(Range.clip(power - strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
            }
        }

        telemetry.addData("angle", current);
        telemetry.addData("target", target);
        telemetry.addData("error", target-current);
        telemetry.addData("power BR", rightBack.getPower());
        telemetry.addData("power BL", leftBack.getPower());
        telemetry.addData("power FR", rightFront.getPower());
        telemetry.addData("power FL", leftFront.getPower());
        telemetry.update();
    }
    public void strafe(double power, int targetHeading, String direction, double inches) throws InterruptedException {
        int ticks = (int)(inches*C*STRAFE_COEFFICIENT);

        resetMotors();
        //double startingPosition = perpendicularEncoderTracker.getCurrentPosition();

        while (motorsBusy(ticks)) {
            correction(power, targetHeading, direction, false);
            heartbeat();
            telemetry.addData("leftBack ticks", Math.abs(leftBack.getCurrentPosition()));
            telemetry.addData("target", ticks);
            telemetry.update();
        }
        halt();
    }


}

