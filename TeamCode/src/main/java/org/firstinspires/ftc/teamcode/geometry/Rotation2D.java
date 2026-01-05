package org.firstinspires.ftc.teamcode.geometry;

/***
 * The goal of this class is to replace Rotation2d, which always sends new objects instead of mutating itself.
 * Rotation2d is being replaced with a copy of itself, Rotation2D, that is mutable.
 */
public class Rotation2D implements IRotatable<Rotation2D>{

    double rot;

    public Rotation2D(){
        this(0);
    }

    public Rotation2D(double angle){
        rot = angle;
        bind2pi();
    }
    public Rotation2D(double x, double y) {
        this(Math.atan2(y, x)); //Yes, does take y, x
    }


    /***
     * All the getters for the units of rotation
     * getRot -> Rotation2D
     * getRad -> double Radians
     * getDeg -> double Degrees
     */
    @Override
    public Rotation2D getRot() {
        return this;
    }

    @Override
    public double getRad() {
        return rot;
    }

    @Override
    public double getDeg() {
        return Math.toDegrees(rot);
    }


    /***
     * All the setters for the units of rotation
     * setRot(newRot) -> rot is now Rotation2D's Radians
     * setRad(rad) -> rot is now rad Radians
     * setDeg(deg) -> rot is now deg Degrees
     */
    @Override
    public Rotation2D setRot(Rotation2D newRot){
        rot = newRot.getRad();
        bind2pi();
        return this;
    }

    @Override
    public Rotation2D setRad(double rad) {
        rot = rad;
        bind2pi();
        return this;

    }

    @Override
    public Rotation2D setDeg(double deg) {
        rot = Math.toRadians(deg);
        bind2pi();
        return this;

    }


    /***
     * Rotates self by the units of rotation
     * rotateBy(newRot) -> newRot's angle is added to self's angle in Radians
     * rotateByRad(rad) -> rad Radians is added to self's angle in Radians
     * rotateByDeg(deg) -> deg Degrees is added to self's angle in Radians
     */
    @Override
    public Rotation2D rotateBy(Rotation2D newRot) {
        rot += newRot.getRad();
        bind2pi();
        return this;
    }

    @Override
    public Rotation2D rotateByRad(double rad) {
        rot += rad;
        bind2pi();
        return this;
    }

    @Override
    public Rotation2D rotateByDeg(double deg) {
        rot += Math.toRadians(deg);
        bind2pi();
        return this;
    }


    /***
     * Simple sin and cos getters, quality of life.
     */
    public double getSin(){
        return Math.sin(rot);
    }

    public double getCos(){
        return Math.cos(rot);
    }


    /***
     * Binds the possible angles to 0 - 2pi, it's a bit funky, but logic says it works.
     * Make sure rot is at least 2pi away
     * Add 2pi, and modulo again
     */
    public void bind2pi() {
        rot = (rot % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
    }

    public Rotation2D copy(){
        return new Rotation2D(rot);
    }

    @Override
    public String toString(){
        return String.format("Rotation2D(rot = %.3f rad)", rot);
    }
}