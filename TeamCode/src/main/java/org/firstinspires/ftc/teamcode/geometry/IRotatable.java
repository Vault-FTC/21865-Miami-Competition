package org.firstinspires.ftc.teamcode.geometry;

public interface IRotatable<T extends IRotatable<T>> {

    default void customMethod(){};

    Rotation2D getRot();

    default Rotation2D getRotCopy(){
        customMethod();
        return getRot().getRotCopy();
    }

    default double getRad(){
        customMethod();
        return getRot().getRad();
    }

    default double getDeg(){
        customMethod();
        return getRot().getDeg();
    }

    default T setRot(Rotation2D newRot){
        customMethod();
        getRot().setRot(newRot);
        return this.copy();
    }

    default T setRad(double radians){
        customMethod();
        getRot().setRad(radians);
        return this.copy();
    }

    default T setDeg(double degrees){
        customMethod();
        getRot().setDeg(degrees);
        return this.copy();
    }

    default T rotateBy(Rotation2D newRot){
        customMethod();
        getRot().rotateBy(newRot);
        return this.copy();
    }

    default T rotateByRad(double rad){
        customMethod();
        getRot().rotateByRad(rad);
        return this.copy();
    }

    default T rotateByDeg(double deg){
        customMethod();
        setRot(getRot().rotateByDeg(deg));
        return copy();
    }

    T copy();
}
