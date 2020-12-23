#!/usr/bin/env python
import Vec3D
import Bezier_curves


def Bezier(x_raw,y_raw,z_raw):
    
    raw_points = []
    count = len(x_raw)

    for i in range(len(x_raw)):
        raw_points.append(Vec3D.Vec3D(x_raw[i],y_raw[i],z_raw[i]))
    
    curve = Bezier_curves.BezierCurve()      
    for index in range(0,count):
        curve.append_point(raw_points[index])
    
    #Compute smoothed points
    c = curve.draw()

    #Smooth x,y,z coordinates for Visualisation
    x_smoothed, y_smoothed, z_smoothed = [], [], []

    for point in c:
        x_smoothed.append(point.x)
        y_smoothed.append(point.y)
        z_smoothed.append(point.z)

    return x_smoothed, y_smoothed, z_smoothed

