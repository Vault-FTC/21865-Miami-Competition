package org.firstinspires.ftc.teamcode.geometry;

public interface IVectorizable< T extends IVectorizable<T> > extends IRotatable<T> {
    Vector2D getVector();

    default Vector2D getVectorCopy(){
        customMethod();
        return getVector().getVectorCopy();
    }

    default double getMag(){
        customMethod();
        return getVector().getMag();
    }

    default double getX(){
        customMethod();
        return getVector().getX();
    }

    default double getY(){
        customMethod();
        return getVector().getY();
    }

    default T mult(double scale){
        customMethod();
        getVector().mult(scale);
        return this.copy();
    }

    default T setMag(double scale){
        customMethod();
        getVector().mult(scale);
        return this.copy();
    }

    default T setVector(Vector2D newVect){
        customMethod();
        getVector().setVector(newVect);
        return this.copy();
    }

    default T addVector(Vector2D newVector){
        customMethod();
        getVector().addVector(newVector);
        return this.copy();
    }

    default T addVector(double x, double y){
        customMethod();
        getVector().addVector(x, y);
        return this.copy();
    }

    T copy();
}
