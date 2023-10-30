package com.team2383.lib.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

public class MotionModel implements BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> {
    private SimpleMatrix F_x;

    public MotionModel(SimpleMatrix F_x) {
        this.F_x = F_x;
    }

    @Override
    public SimpleMatrix apply(SimpleMatrix u, SimpleMatrix mu) {
        double dx = u.get(0);
        double dy = u.get(1);
        double dtheta = u.get(2);

        double rw = mu.get(3);
        double rx = mu.get(4);
        double ry = mu.get(5);
        double rz = mu.get(6);

        SimpleMatrix g = new SimpleMatrix(7, 1);

        g.set(0, rw * (dx * rw - dy * rz) + rx * (dx * rx + dy * ry) - ry * (dx * ry - dy * rx)
                - rz * (dx * rz + dy * rw));

        g.set(1, rw * (dx * rz + dy * rw) + rx * (dx * ry - dy * rx) + ry * (dx * rx + dy * ry)
                + rz * (dx * rw - dy * rz));

        g.set(2, -2 * dx * rw * ry + 2 * dx * rx * rz + 2 * dy * rw * rx + 2 * dy * ry * rz);

        g.set(3, (-rw * Math.sqrt(Math.pow(rw, 2) + Math.pow(rz, 2)) + rw * Math.cos((1.0 / 2.0) * dtheta)
                - rz * Math.sin((1.0 / 2.0) * dtheta)) / Math.sqrt(Math.pow(rw, 2) + Math.pow(rz, 2)));

        g.set(4, -rx);

        g.set(5, -ry);

        g.set(6, (rw * Math.sin((1.0 / 2.0) * dtheta) - rz * Math.sqrt(Math.pow(rw, 2) + Math.pow(rz, 2))
                + rz * Math.cos((1.0 / 2.0) * dtheta)) / Math.sqrt(Math.pow(rw, 2) + Math.pow(rz, 2)));

        return F_x.transpose().mult(g);
    }
}
