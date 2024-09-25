package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Robot extends BaseHardware {

    public DriveTrain driveTrain = new DriveTrain();
    public Lighting lighting = new Lighting();
    public Sensors sensors = new Sensors();
    public Arm arm = new Arm();
    public Intake intake = new Intake();

    @Override
    public void init() {
        // Must set Hardware Map and telemetry before calling init
        driveTrain.hardwareMap = this.hardwareMap;
        driveTrain.telemetry = this.telemetry;
        driveTrain.init();

           lighting.hardwareMap = this.hardwareMap;
        lighting.telemetry = this.telemetry;
        lighting.init();

        sensors.hardwareMap = this.hardwareMap;
        sensors.telemetry = this.telemetry;
        sensors.init();

        arm.hardwareMap = this.hardwareMap;
        arm.telemetry = this.telemetry;
        arm.init();

        intake.hardwareMap = this.hardwareMap;
        intake.telemetry = this.telemetry;
        intake.init();


    }

    @Override
    public void init_loop() {
        driveTrain.init_loop();
        lighting.init_loop();
        sensors.init_loop();
        arm.init_loop();
        intake.init_loop();

    }

    @Override
    public void start() {
        driveTrain.start();
        lighting.start();
        sensors.start();
        arm.start();
        intake.start();

        lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    @Override
    public void loop() {
        driveTrain.loop();
        lighting.loop();
        sensors.loop();
        arm.loop();
        intake.loop();
    }


    @Override
    public void stop() {
        driveTrain.stop();
        lighting.stop();
        sensors.stop();
        arm.stop();
        intake.stop();

        lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }




}
