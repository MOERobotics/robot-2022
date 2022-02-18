package frc.robot.generic;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class Lightning implements GenericRobot {
    public static final double TICKS_PER_INCH_DRIVE = 116;
    public static final double TICKS_PER_DEGREE_TURRET = 116;
    public static final double TICKS_PER_REVOLUTION_SHOOTERA = 116;
    public static final double TICKS_PER_REVOLUTION_SHOOTERB = 116;

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

    RelativeEncoder encoderRight  = rightMotorA.getEncoder();
    RelativeEncoder encoderLeft   = leftMotorA.getEncoder();
    RelativeEncoder encoderTurret = turretRotator.getEncoder();
    RelativeEncoder encoderShootA = shooterA.getEncoder();
    RelativeEncoder encoderShootB = shooterB.getEncoder();

    //DigitalInput ballSensor = new DigitalInput(0);
    DoubleSolenoid PTO = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    DoubleSolenoid arms = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    Solenoid collectorPosition = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    SparkMaxPIDController shooterAPIDController = shooterA.getPIDController();
    SparkMaxPIDController shooterBPIDController = shooterB.getPIDController();

    public Lightning(){
        boolean invertLeft = false;
        boolean invertRight = true;
        leftMotorA.setInverted(invertLeft);
        leftMotorB.setInverted(invertLeft);
        rightMotorA.setInverted(invertRight);
        rightMotorB.setInverted(invertRight);

        indexer.setInverted(true);
        collector.setInverted(false);
        shooterB.setInverted(true);
        shooterA.setInverted(false);
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
        return GenericRobot.super.getUpperCargo();
    }
    @Override
    public boolean getLowerCargo() {
        return GenericRobot.super.getLowerCargo();
    }


    @Override
    public void setCollectorIntakePercentage(double percentage) {
        collector.set(percentage);
        indexer.set(percentage);
    }

    @Override
    public double getCollectorIntakePercentage() {
        return collector.get();
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
    public void raiseCollector(){
        collectorPosition.set(true);
    }

    @Override
    public void lowerCollector(){
        collectorPosition.set(false);
    }

    @Override
    public void turnOnPTO(){
        PTO.set(DoubleSolenoid.Value.kForward);
    }
    @Override
    public void turnOffPTO(){
        PTO.set(DoubleSolenoid.Value.kReverse);
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
    public void raiseClimberArms() {
        leftMotorB.set(.1);
        leftMotorA.set(.1);
        rightMotorA.set(.1);
        rightMotorB.set(.1);
    }

    @Override
    public void lowerClimberArms() {
        leftMotorB.set(-.1);
        leftMotorA.set(-.1);
        rightMotorA.set(-.1);
        rightMotorB.set(-.1);
    }

    @Override
    public void armPower(double power) {
        leftMotorB.set(power);
        leftMotorA.set(power);
        rightMotorA.set(power);
        rightMotorB.set(power);
    }

    @Override
    public double armHeight() {
        //TODO
        //Maybe use some sensor. Do NOT want to use encoders for this.
        return GenericRobot.super.armHeight();
    }

    @Override
    public boolean armInContact() {
        //TODO: find tolerances
        if (leftMotorA.getOutputCurrent() > LEFTATOLERANCE && leftMotorB.getOutputCurrent() > LEFTBTOLERANCE
                && rightMotorA.getOutputCurrent() > RIGHTATOLERANCE && rightMotorB.getOutputCurrent() > RIGHTBTOLERANCE){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public boolean inTheRightPlace(){
        //TODO: maybe a sensor??
        return false;
    }

    @Override
    public double getPIDmaneuverP() {
        return GenericRobot.super.getPIDmaneuverP();
    }

    @Override
    public double getPIDmaneuverI() {
        return GenericRobot.super.getPIDmaneuverI();
    }

    @Override
    public double getPIDmaneuverD() {
        return GenericRobot.super.getPIDmaneuverD();
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
    public boolean getTapeSensorOne() {
        return false;
    }

    @Override
    public boolean getTapeSensorTwo() {
        return false;
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
}
