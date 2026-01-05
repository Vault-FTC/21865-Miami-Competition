package org.firstinspires.ftc.teamcode.geometry;

/***
 * The goal of Vector2D is to have a Vector that is constrained between angles 0 - 2pi and has a positive magnitude.
 * This object's methods should only mutate itself. NOT return a copy of itself (unless using copy).
 * For practical sake, multiplying by a negative number works, you will have a positive vector in the opposite direction.
 */
public class Vector2D implements IVectorizable<Vector2D> {

    private Rotation2D rot;
    private double mag;

    public Vector2D(){
        this(0, new Rotation2D());
    }

    public Vector2D(double mag, Rotation2D rot){
        this.rot = rot;
        this.mag = mag;
    }

    public Vector2D(double x, double y){
        this(Math.hypot(x, y), new Rotation2D(Math.atan2(y, x)));
    }

    /**
     * This method is called in IVectorizable! This is the object we are mutating!
     * @returns itself.
     */
    @Override
    public Vector2D getVector() {
        return this;
    }

    /**
     * This method is called in IRotatable! This is the object we are mutating!
     * @returns the vector's rotation.
     */
    @Override
    public Rotation2D getRot() {
        return rot;
    }

    /**
     * Other getters. No mutable references are passed.
     */
    @Override
    public Vector2D getVectorCopy(){
        return copy();
    }

    @Override
    public Rotation2D getRotCopy() {
        return rot.copy();
    }

    @Override
    public double getX(){
        return mag * rot.getCos();
    }

    @Override
    public double getY(){
        return mag * rot.getSin();
    }

    @Override
    public double getMag(){
        return mag;
    }

    /**
     * These are the setters for the vector. Self-explanatory.
     */
    @Override
    public Vector2D mult(double scale){
        mag *= scale;
        bindPos();
        return copy();
    }

    @Override
    public Vector2D setMag(double newMag){
        mag = newMag;
        bindPos();
        return copy();
    }

    @Override
    public Vector2D setVector(Vector2D newVect){
        rot = newVect.getRotCopy();
        mag = newVect.getMag();
        return copy();
    }

    @Override
    public Vector2D addVector(Vector2D newVector){
        return addVector(newVector.getX(), newVector.getY());
    }

    @Override
    public Vector2D addVector(double x, double y){
        if(Math.hypot(x, y) < 1e-6) {
            return copy();
        }
        x += this.getX();
        y += this.getY();
        setMag(Math.hypot(x, y));
        setRad(Math.atan2(y, x));
        bindPos();
        return copy();
    }

    /**
     * In this case, the same as getVectorCopy. This is a compromise for using the IVectorizable interface for other features.
     * @returns A copy of itself.
     */
    @Override
    public Vector2D copy(){
        return new Vector2D(mag, rot.copy());
    }

    /**
     * Keeps the vector positive. Checked whenever a vector is altered.
     */
    public void bindPos(){
        if (mag < 0) {
            rot.rotateByDeg(180);
            mag *= -1;
        }
    }

    @Override
    public String toString() {
        return String.format("Vector2D(mag=%.3f, angle=%.3f rad, x=%.3f, y=%.3f)",
                mag, rot.getRad(), getX(), getY());
    }
}
