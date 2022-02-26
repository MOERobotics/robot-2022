package frc.robot.generic;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class Lightning implements GenericRobot {
    public static final double TICKS_PER_INCH_DRIVE = 0.96;
    public static final double TICKS_PER_DEGREE_TURRET = 116;
    public static final double TICKS_PER_DEGREE_TURRET2 = 136.467;
    public static final double TICKS_PER_REVOLUTION_SHOOTERA = 1;
    public static final double TICKS_PER_REVOLUTION_SHOOTERB = 1;

    public static  final double LEFTATOLERANCE = 0;
    public static  final double LEFTBTOLERANCE = 0;
    public static  final double RIGHTATOLERANCE = 0;
    public static  final double RIGHTBTOLERANCE = 0;

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    CANSparkMax collector         = new CANSparkMax( 3, kBrushless);
    CANSparkMax indexer           = new CANSparkMax( 4, kBrushless);
    CANSparkMax turretRotator     = new CANSparkMax( 2, kBrushless);
    CANSparkMax shooterA          = new CANSparkMax( 5, kBrushless);
    CANSparkMax shooterB          = new CANSparkMax( 6, kBrushless);

    CANSparkMax leftMotorA        = new CANSparkMax(20, kBrushless);
    CANSparkMax leftMotorB        = new CANSparkMax( 1, kBrushless);
    CANSparkMax rightMotorA       = new CANSparkMax(18, kBrushless);
    CANSparkMax rightMotorB       = new CANSparkMax(19, kBrushless);

    //TODO: update servo ports
    //servo left was initially set to channel 9, don't know if that means anything
    Servo       elevationLeft     = new Servo(9);
    Servo       elevationRight     = new Servo(1);


    RelativeEncoder encoderRight  = rightMotorA.getEncoder();
    RelativeEncoder encoderLeft   = leftMotorA.getEncoder();
    RelativeEncoder encoderTurret = turretRotator.getEncoder();
    SparkMaxAnalogSensor encoderTurretAlt = turretRotator.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    RelativeEncoder encoderShootA = shooterA.getEncoder();
    RelativeEncoder encoderShootB = shooterB.getEncoder();


    //DigitalInput ballSensor = new DigitalInput(0);
    DoubleSolenoid PTO = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    DoubleSolenoid arms = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    DoubleSolenoid collectorPosition = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6,7);

    SparkMaxPIDController shooterAPIDController = shooterA.getPIDController();
    SparkMaxPIDController shooterBPIDController = shooterB.getPIDController();

    SparkMaxLimitSwitch.Type lstype = SparkMaxLimitSwitch.Type.kNormallyClosed;

    SparkMaxLimitSwitch limitSwitchIndexerForward = indexer.getForwardLimitSwitch(lstype);
    SparkMaxLimitSwitch limitSwitchIndexerReverse = indexer.getReverseLimitSwitch(lstype);

    SparkMaxLimitSwitch limitSwitchRightAForward = rightMotorA.getForwardLimitSwitch(lstype);
    SparkMaxLimitSwitch limitSwitchRightAReverse = rightMotorA.getReverseLimitSwitch(lstype);

    SparkMaxLimitSwitch limitSwitchLeftBForward = leftMotorB.getForwardLimitSwitch(lstype);
    SparkMaxLimitSwitch limitSwitchLeftBReverse = leftMotorB.getReverseLimitSwitch(lstype);

    double defaultShooterTargetRPM = 5000;

    boolean isPTOonArms;

    //True = robot is in the process of and committed to shooting a cargo at mach 12
    boolean isActivelyShooting;
    boolean canShoot;

    //shootReadyTimer is used to check if shooter ready
    long shootReadyTimer;

    public Lightning(){
        boolean invertLeft = false;
        boolean invertRight = true;
        leftMotorA.setInverted(invertLeft);
        leftMotorB.setInverted(invertLeft);
        rightMotorA.setInverted(invertRight);
        rightMotorB.setInverted(invertRight);

        leftMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotorA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotorB.setIdleMode(CANSparkMax.IdleMode.kBrake);

        indexer.setInverted(true);
        collector.setInverted(false);
        shooterB.setInverted(false);
        shooterA.setInverted(true);

        elevationLeft.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        elevationRight.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);


        shootReadyTimer = System.currentTimeMillis();

        isPTOonArms = false;
        isActivelyShooting = false;

        indexer.setIdleMode(CANSparkMax.IdleMode.kBrake);

        limitSwitchIndexerForward.enableLimitSwitch(false);
        limitSwitchIndexerReverse.enableLimitSwitch(false);
        limitSwitchRightAForward.enableLimitSwitch(false);
        limitSwitchRightAReverse.enableLimitSwitch(false);

        limitSwitchLeftBForward.enableLimitSwitch(false);
        limitSwitchLeftBReverse.enableLimitSwitch(false);

    }

    @Override
    public void drivePercent(double leftPercent, double rightPercent) {
        leftMotorA .set( leftPercent);
        leftMotorB .set( leftPercent);
        rightMotorA.set(rightPercent);
        rightMotorB.set(rightPercent);
    }

    @Override
    public double getDriveLeftPercentage() {
        return leftMotorA.get();
    }

    @Override
    public double getDriveRightPercentage() {
        return rightMotorA.get();
    }

    @Override
    public double getDriveLeftRPM() {
        return encoderLeft.getVelocity()/encoderLeftDriveTicksPerInch();
    }

    @Override
    public double getDriveRightRPM() {
        return encoderRight.getVelocity()/encoderRightDriveTicksPerInch();
    }

    @Override
    public double getLeftACurrent(){
        return leftMotorA.getOutputCurrent();
    }

    @Override
    public double getLeftBCurrent(){
        return leftMotorB.getOutputCurrent();
    }

    @Override
    public double getRightACurrent(){
        return rightMotorA.getOutputCurrent();
    }

    @Override
    public double getRightBCurrent(){
        return rightMotorB.getOutputCurrent();
    }

    @Override
    public double encoderLeftDriveTicksPerInch() {
        return TICKS_PER_INCH_DRIVE;
    }

    @Override
    public double encoderRightDriveTicksPerInch() {
        return TICKS_PER_INCH_DRIVE;
    }

    @Override
    public double encoderTicksLeftDrive() {
        return encoderLeft.getPosition();
    }

    @Override
    public double encoderTicksRightDrive() {
        return encoderRight.getPosition();
    }

    @Override
    public double getYaw() {
        return navx.getYaw();
    }

    @Override
    public double getPitch() {
        return navx.getPitch();
    }

    @Override
    public double getRoll() {
        return navx.getRoll();
    }

    @Override
    public boolean getUpperCargo() {
        return limitSwitchIndexerReverse.isPressed();
    }
    @Override
    public boolean getLowerCargo() {
        return limitSwitchIndexerForward.isPressed();
    }

    @Override
    public void getCargo(){
        double collectorPct = 1;
        double indexerPct = 1;
        if (getUpperCargo()){
            if (isActivelyShooting){
                indexerPct = 1;
            }
            else{
                indexerPct = 0;
            }
            if (getLowerCargo()){
                collectorPct = 0;
            }
        }
        setIndexerIntakePercentage(indexerPct);
        setCollectorIntakePercentage(collectorPct);

    }

    @Override
    public void shoot(){
        double shooterRPM = defaultShooterTargetRPM;
        if (getShooterRPMTop() >= shooterRPM && getShooterRPMBottom() >= shooterRPM){
            canShoot = true;
        }
        else{
            canShoot = false;
        }
    }

    @Override
    public boolean canShoot(){
        return canShoot;
    }


    @Override
    public void setCollectorIntakePercentage(double percentage) {
        collector.set(percentage);
    }

    @Override
    public double getCollectorIntakePercentage() {
        return collector.get();
    }

    @Override
    public void setIndexerIntakePercentage(double percentage) {
        indexer.set(percentage);
    }

    @Override
    public double getIndexerIntakePercentage() {
        return indexer.get();
    }

    @Override
    public boolean hasFoundCargo() {
        return GenericRobot.super.hasFoundCargo();
    }

    @Override
    public TeamColor getFoundCargoColor() {
        return GenericRobot.super.getFoundCargoColor();
    }

    @Override
    public boolean isTargetFound() {
        return GenericRobot.super.isTargetFound();
    }


    @Override
    public double getTurretAngle() {
        return encoderTurret.getPosition();
    }

    @Override
    public double encoderTurretTicksPerDegree(){
        return TICKS_PER_DEGREE_TURRET;
    }

    //Measured range 0.042 - 2.68
    @Override
    public double getAlternateTurretAngle(){
        double raw = encoderTurretAlt.getPosition();
        return (raw *  136.467) - 5.73;
    }

    @Override
    public void setTurretAngleRelative(double angleChange) {
        //TODO

    }

    @Override
    public void setTurretAngleAbsolute() {
        //TODO

    }



    @Override
    public void setTurretPowerPct(double powerPct) {
        turretRotator.set(powerPct);
    }

    @Override
    public double getTurretPowerPct() {
        return turretRotator.get();
    }

    @Override
    public double getTurretPitchPosition(){

        //TODO: getSpeed()? getPosition()? getAngle()? don't know which to use
        return elevationLeft.getSpeed();
    }
    @Override
    public void setTurretPitchPosition(double position){
        if(position < 0) position = 0;
        if(position > 1) position = 1;


        //TODO: figure out use setSpeed() or set()
        elevationLeft.set(position);
        elevationRight.set(position);
    }


    @Override
    public double getShooterRPMTop() {
        return encoderShootA.getVelocity() / TICKS_PER_REVOLUTION_SHOOTERA;
    }

    @Override
    public double getShooterRPMBottom() {
        return encoderShootB.getVelocity() / TICKS_PER_REVOLUTION_SHOOTERB;
    }

    @Override
    public double getShooterPowerPctTop() {
        return shooterA.get();
    }

    @Override
    public double getShooterPowerPctBottom() {
        return shooterB.get();
    }

    @Override
    public double getShooterTargetDistance() {
        //TODO
        return 0;
    }

    @Override
    public double getShooterTargetHeight() {
        //TODO
        return 0;
    }


    @Override
    public void setShooterRPM(double topRPM, double bottomRPM) {
        setShooterRPMTop(topRPM);
        setShooterRPMBottom(bottomRPM);
    }

    @Override
    public void setShooterRPMTop(double rpm) {
        shooterAPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setShooterRPMBottom(double rpm) {
        shooterBPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }



    @Override
    public void setShooterPowerPct(double topPCT, double bottomPCT) {
        setShooterPowerPctTop(topPCT);
        setShooterPowerPctBottom(bottomPCT);
    }

    @Override
    public void setShooterPowerPctTop(double percentage) {
        shooterA.set(percentage);
    }

    @Override
    public void setShooterPowerPctBottom(double percentage) {
        shooterB.set(percentage);

    }

    @Override
    public void setShooterTargetDistance(double length, double height) {
        //TODO
    }

    @Override
    public double getShooterTargetRPM(){
        return defaultShooterTargetRPM;
    }

    @Override
    public void setShooterTargetRPM(double rpm){
        defaultShooterTargetRPM = rpm;
    }

    @Override
    public boolean getFloorSensorLeft(){
        return limitSwitchLeftBForward.isPressed();

    }
    @Override
    public boolean getFloorSensorRight(){
        return limitSwitchRightAForward.isPressed();

    }

    @Override
    public void raiseCollector(){
        collectorPosition.set(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void lowerCollector(){
        collectorPosition.set(DoubleSolenoid.Value.kReverse);
    }

    //CONNECTS MOTORS TO CLIMB ARMS
    @Override
    public void turnOnPTO(){
        isPTOonArms = true;
        PTO.set(DoubleSolenoid.Value.kForward);
    }

    //CONNECTS MOTORS TO DRIVE
    @Override
    public void turnOffPTO(){
        isPTOonArms = false;
        PTO.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public boolean getPTOState(){
        return isPTOonArms;
    }

    @Override
    public void setArmsForward(){
        arms.set(DoubleSolenoid.Value.kReverse);
    }
    @Override
    public void setArmsBackward(){
        arms.set(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void armPower(double leftPower, double rightPower) {
        leftMotorB.set(leftPower);
        leftMotorA.set(leftPower);
        rightMotorA.set(rightPower);
        rightMotorB.set(rightPower);
    }
    @Override
    public void raiseClimberArms(double rightPower, double leftPower){
        //System.out.println("I don't have a climber");
        armPower(leftPower, rightPower);
    }
    @Override
    public void lowerClimberArms(double rightPower, double leftPower){
        //System.out.println("I don't have a climber");
        armPower(-leftPower, -rightPower);
    }

    @Override
    public double armHeightLeft() {
        //TODO: put in conversion
        //Maybe use some sensor. Do NOT want to use encoders for this.
        return -encoderTicksLeftDrive()*.24937;
    }

    @Override
    public double armHeightRight(){
        //TODO: put in conversion
        return -encoderTicksRightDrive()*.24937;
    }

    @Override
    public boolean armInContact() {
        //TODO: find tolerances
       /* if (leftMotorA.getOutputCurrent() > LEFTATOLERANCE && leftMotorB.getOutputCurrent() > LEFTBTOLERANCE
                && rightMotorA.getOutputCurrent() > RIGHTATOLERANCE && rightMotorB.getOutputCurrent() > RIGHTBTOLERANCE){
            return true;
        }
        else{
            return false;
        }*/
        return true;
    }



    @Override
    public boolean inTheRightPlace(){
        //TODO: maybe a sensor??
        return false;
    }

    @Override
    public boolean getClimbSensorLeft(){
        return limitSwitchLeftBReverse.isPressed();
    }
    @Override
    public boolean getClimbSensorRight(){
        return limitSwitchRightAReverse.isPressed();
    }

    @Override
    public double getPIDmaneuverP() {
        return 1.0e-2;
    }

    @Override
    public double getPIDmaneuverI() {
        return 0;
    }

    @Override
    public double getPIDmaneuverD() {
        return 1.0e-4;
    }

    @Override
    public double getPIDpivotP() {
        return GenericRobot.super.getPIDpivotP();
    }

    @Override
    public double getPIDpivotI() {
        return GenericRobot.super.getPIDpivotI();
    }

    @Override
    public double getPIDpivotD() {
        return GenericRobot.super.getPIDpivotD();
    }

    @Override
    public void resetEncoders() {
        leftMotorA.getEncoder().setPosition(0);
        leftMotorB.getEncoder().setPosition(0);
        rightMotorA.getEncoder().setPosition(0);
        rightMotorB.getEncoder().setPosition(0);
    }

    @Override
    public void resetAttitude() {
        navx.reset();
    }


    @Override
    public double turretPIDgetP() {
        return GenericRobot.super.turretPIDgetP();
    }

    @Override
    public double turretPIDgetI() {
        return GenericRobot.super.turretPIDgetI();
    }

    @Override
    public double turretPIDgetD() {
        return GenericRobot.super.turretPIDgetD();
    }


    @Override
    public long getShootReadyTimer(){
        return shootReadyTimer;
    }
    @Override
    public void shooterNotReady(){
        shootReadyTimer = System.currentTimeMillis();
    }

    @Override
    public boolean isActivelyShooting(){
        return isActivelyShooting;
    }

    //TODO: Add check using isReadyToShoot() function?
    @Override
    public void setActivelyShooting(boolean isShooting){
        isActivelyShooting = isShooting;
    }
}
