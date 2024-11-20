package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Robot extends BaseHardware {

    private final pickupTypeEnum PickupType = pickupTypeEnum.INTAKE_CLAW;
    public DriveTrain driveTrain = new DriveTrain();
    //public Lighting lighting = new Lighting();
    public Sensors sensors = new Sensors();
    public Arm arm = new Arm();
    public Intake intake = new Intake();
    public Claw claw = new Claw();



    @Override
    public void init() {
        // Must set Hardware Map and telemetry before calling init
        driveTrain.hardwareMap = this.hardwareMap;
        driveTrain.telemetry = this.telemetry;
        driveTrain.init();

        /*lighting.hardwareMap = this.hardwareMap;
        lighting.telemetry = this.telemetry;
        lighting.init();
*/
        sensors.hardwareMap = this.hardwareMap;
        sensors.telemetry = this.telemetry;
        sensors.init();

        arm.hardwareMap = this.hardwareMap;
        arm.telemetry = this.telemetry;
        arm.init();

        if( PickupType == pickupTypeEnum.INTAKE_SWEEPER ) {
            intake.hardwareMap = this.hardwareMap;
            intake.telemetry = this.telemetry;
            intake.init();
        }
        else {
            claw.hardwareMap = this.hardwareMap;
            claw.telemetry = this.telemetry;
            claw.init();
        }

        claw.hardwareMap = this.hardwareMap;
        claw.telemetry = this.telemetry;
        claw.init();

    }

    @Override
    public void init_loop() {
        driveTrain.init_loop();
        //lighting.init_loop();
        sensors.init_loop();
        arm.init_loop();

        if( PickupType == pickupTypeEnum.INTAKE_SWEEPER ) {
            intake.init_loop();
        }
        else {
            claw.init_loop();
        }


    }

    @Override
    public void start() {
        driveTrain.start();
        //lighting.start();
        sensors.start();
        arm.start();
        if( PickupType == pickupTypeEnum.INTAKE_SWEEPER ) {
            intake.start();
        }
        else {
            claw.start();
        }

        //lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    @Override
    public void loop() {
        driveTrain.loop();
        //lighting.loop();
        sensors.loop();
        arm.loop();
        if( PickupType == pickupTypeEnum.INTAKE_SWEEPER ) {
            intake.loop();
        }
        else {
            claw.loop();
        }

    }


    @Override
    public void stop() {
        driveTrain.stop();
        //lighting.stop();
        sensors.stop();
        arm.stop();
        if( PickupType == pickupTypeEnum.INTAKE_SWEEPER ) {
            intake.stop();
        }
        else {
            claw.stop();
        }

        //lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

enum pickupTypeEnum {
        INTAKE_CLAW,
        INTAKE_SWEEPER;
}


}
