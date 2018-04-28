//
// Created by discontent-civilian on 4/9/18.
//

#ifndef DDP_KRANG2D_H
#define DDP_KRANG2D_H

#include <ddp/costs.hpp>
#include <ddp/dynamics.hpp>
#include <ddp/plant.hpp>
#include <ddp/eigenmvn.hpp>
#include <QtCore>
#include <QVector>

template <class T>
struct Krang2D: public Dynamics<T, 4, 1>
{
    using State = typename Dynamics<T, 4, 1>::State;
    using Control = typename Dynamics<T, 4, 1>::Control;

    Krang2D(T g, T M, T m, T L, T R,T iw, T I) : gravity(g), totalmass(M), masswheel(m), length(L), radius(R), inertiawheel(iw), inertia (I) {}
    inline State f(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u)
    {
        using std::sin; using std::cos;

        // state transition = [thetadot, force, xdot, xddot]
        State dx;

        // state = [theta, thetadot,x,xdot]
        const T force = u.value();
        const T costheta = cos(x(0));
        const T sintheta = sin(x(0));

        const T alpha = ((radius/(totalmass*length))*totalmass+masswheel+(inertiawheel/(radius*radius)));
        const T beta = inertia/(totalmass*length);
        const T xddot = (gravity*sintheta-(radius*sintheta*x(1)*x(1)) - (beta+costheta)*force)/(alpha+costheta);

        dx(0) = x(1);
        dx(1) = force;
        dx(2) = x(3);
        dx(3) = xddot;

        return dx;
//        State delta_x;
//
//        T xx = x(2);
//        T dx = x(3);
//        T q1 = x(0);
//        T dq1 = x(1);
//
//        T g = 9.8;
//        T dt = 1;
//        T MX1 = 0;
//        T R = 0.25;
//        T fric1 = 0.400;
//        T MY1 = 31.4484;
//        T Iw = 0.0051;
//        T m1 = 115.256;
//        T mw = 0.51;
//
//        // somehow the actual length in matlab
//        T L1 = 0.5457;
//
//        // somehow the actual inertia in matlab
//        T I1 = 11.4412;
//
//        // RESULT OF COMPUTE TORQUE
//        T tau1 = u.value();
//
//        // change in theta
//        delta_x(0) = x(1);
//        // change in theta dot
//        delta_x(1) = (dt*(Iw*R*tau1 + Iw*dx*fric1 + pow(R,3)*m1*tau1 + pow(R,3)*mw*tau1 - pow(R,3)*dq1*fric1*m1 + pow(R,2)*dx*fric1*m1 - pow(R,3)*dq1*fric1*mw + pow(R,2)*dx*fric1*mw + MY1*pow(R,2)*tau1*cos(q1) - MX1*pow(R,2)*tau1*sin(q1) - Iw*R*dq1*fric1 + MX1*pow(R,3)*g*m1*cos(q1) + MX1*pow(R,3)*g*mw*cos(q1) + MX1*pow(R,2)*dq1*fric1*sin(q1) + MY1*pow(R,3)*g*m1*sin(q1)+ MY1*pow(R,3)*g*mw*sin(q1) + pow(MX1,2)*pow(R,3)*pow(dq1,2)*cos(q1)*sin(q1) - pow(MY1,2)*pow(R,3)*pow(dq1,2)*cos(q1)*sin(q1) + Iw*MX1*R*g*cos(q1) + Iw*MY1*R*g*sin(q1) + MY1*R*dx*fric1*cos(q1) - MX1*MY1*pow(R,3)*pow(dq1,2)*pow(cos(q1),2) - MX1*R*dx*fric1*sin(q1) + MX1*MY1*pow(R,3)*pow(dq1,2)*pow(sin(q1),2) - MY1*pow(R,2)*dq1*fric1*cos(q1)))/(R*(I1*Iw - pow(MX1,2)*pow(R,2)*pow(sin(q1),2) + I1*pow(R,2)*m1 + I1*pow(R,2)*mw - pow(MY1,2)*pow(R,2)*pow(cos(q1),2) + 2*MX1*MY1*pow(R,2)*cos(q1)*sin(q1)));
//        // change in x
//        delta_x(2) = x(3);
//        // change in x dot
//        delta_x(3) = (dt*(MX1*pow(R, 2)*tau1*sin(q1) - I1*dx*fric1 - MY1*pow(R, 2)*tau1*cos(q1) - I1*R*tau1 + I1*R*dq1*fric1 - MX1*pow(R, 2)*dq1*fric1*sin(q1) + I1*MX1*pow(R, 2)*pow(dq1, 2)*cos(q1) - MX1*MY1*pow(R,2)*g*pow(cos(q1), 2) + I1*MY1*pow(R,2)*pow(dq1,2)*sin(q1) + MX1*MY1*pow(R,2)*g*pow(sin(q1),2) - MY1*R*dx*fric1*cos(q1) + MX1*R*dx*fric1*sin(q1) + pow(MX1,2)*pow(R,2)*g*cos(q1)*sin(q1) - pow(MY1,2)*pow(R,2)*g*cos(q1)*sin(q1) + MY1*pow(R,2)*dq1*fric1*cos(q1)))/(I1*Iw - pow(MX1,2)*pow(R,2)*pow(sin(q1),2) + I1*pow(R,2)*m1 + I1*pow(R,2)*mw - pow(MY1,2)*pow(R,2)*pow(cos(q1),2) + 2*MX1*MY1*pow(R,2)*cos(q1)*sin(q1));
//
//
//        return delta_x;

    }

    T gravity;
    T totalmass;
    T masswheel;
    T length;
    T radius;
    T inertia;
    T inertiawheel;
};

template <class T>
struct Krang2DCost: public CostFunction<Krang2D<T>>
{
    using Scalar = T;
    using Dynamics = Krang2D<T>;
    using State = typename CostFunction<Krang2D<T>>::State;
    using Control = typename CostFunction<Krang2D<T>>::Control;
    using Gradient = typename CostFunction<Krang2D<T>>::Gradient;
    using Hessian = typename CostFunction<Krang2D<T>>::Hessian;
    using StateHessian = Eigen::Matrix<Scalar, Krang2D<T>::StateSize, Krang2D<T>::StateSize>;
    using ControlHessian = Eigen::Matrix<Scalar, Krang2D<T>::ControlSize, Krang2D<T>::ControlSize>;

    Krang2DCost(const Eigen::Ref<const State> &xf, const Eigen::Ref<const StateHessian> &Q, const Eigen::Ref<const ControlHessian> &R)
            : CostFunction<Krang2D<T>>(xf), Q_(Q), R_(R)
    {
        QR_.setZero();
        QR_.topLeftCorner(Dynamics::StateSize, Dynamics::StateSize) = Q;
        QR_.bottomRightCorner(Dynamics::ControlSize, Dynamics::ControlSize) = R;
    }

    Scalar c(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u)
    {

        StateHessian Q_local = Q_;
        State error = x - this->target();
        // overshooting
//        if (error(2) > 0) {
//            Q_local(2, 2) = 100;
//        }
        return (error.transpose() * Q_local * error).value() + (u.transpose() * R_ * u).value();
    }

    Gradient dc(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u)
    {
        Gradient g;
        StateHessian Q_local = Q_;
        State error = x - this->target();
        // overshooting
//        if (error(2) > 0) {
//            Q_local(2, 2) = 100;
//        }
        g.head(Dynamics::StateSize) = Q_local * (x - this->target());
        g.tail(Dynamics::ControlSize) = R_ * u;
        return g;
    }

    Hessian d2c(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u)
    {
        return QR_;
    }

    // Cost Functions which takes reference instead of target
    Scalar c_ref(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u, const Eigen::Ref<const State> &xf)
    {
        State error = x - xf;
        return (error.transpose() * Q_ * error).value() + (u.transpose() * R_ * u).value();
    }

    Gradient dc_ref(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u, const Eigen::Ref<const State> &xf)
    {
        Gradient g;
        g.head(Dynamics::StateSize) = Q_ * (x - xf);
        g.tail(Dynamics::ControlSize) = R_ * u;
        return g;
    }

    Hessian d2c_ref(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u, const Eigen::Ref<const State> &xf)
    {
        return QR_;
    }


private:
    StateHessian Q_;
    ControlHessian R_;
    Hessian QR_;
};

template <class T>
struct Krang2DTerminalCost: public TerminalCostFunction<Krang2D<T>>
{
    using Scalar = T;
    using Dynamics = Krang2D<T>;
    using State = typename TerminalCostFunction<Krang2D<T>>::State;
    using Gradient = typename TerminalCostFunction<Krang2D<T>>::Gradient;
    using Hessian = typename TerminalCostFunction<Krang2D<T>>::Hessian;

    Krang2DTerminalCost(const Eigen::Ref<const State> &xf, const Eigen::Ref<const Hessian> &Q)
            : TerminalCostFunction<Krang2D<T>>(xf), Q_(Q) {}

    Scalar c(const Eigen::Ref<const State> &x)
    {
        return (x - TerminalCostFunction<Krang2D<T>>::xf).transpose() *
               Q_ * (x - TerminalCostFunction<Krang2D<T>>::xf);
    }

    Gradient dc(const Eigen::Ref<const State> &x)
    {
        return Q_ * (x - TerminalCostFunction<Krang2D<T>>::xf);
    }

    Hessian d2c(const Eigen::Ref<const State> &x)
    {
        return Q_;
    }

private:
    Hessian Q_;
};

struct Krang2DPlant: public QObject, public Plant<double, 4, 1>
{
    using Base                  = Plant<double, 4, 1>;
    using Scalar                = typename Base::Scalar;
    using State                 = typename Base::State;
    using Control               = typename Base::Control;
    using StateNoiseVariance    = Eigen::Matrix<Scalar, StateSize, StateSize>;
    using ControlNoiseVariance  = Eigen::Matrix<Scalar, ControlSize, ControlSize>;

    Krang2DPlant(Krang2D<Scalar> &cp, Scalar dt, Scalar state_var, Scalar control_var, QObject *parent = 0)
            : QObject(parent), cp_(cp), dt_(dt), sdist_(State::Zero(), state_var * StateNoiseVariance::Identity()),
              cdist_(Control::Zero(), control_var * ControlNoiseVariance::Identity()) {}

    inline State f(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u)
    {
        static QVector<Scalar> sx(StateSize);
        static QVector<Scalar> su(ControlSize);
        static QVector<Scalar> sx_ref(StateSize);

        Control u_noisy = u + cdist_.samples(1);

        State xnew = x + cp_.f(x, u_noisy) * dt_ + sdist_.samples(1);
        Eigen::Map<State>(sx.data(), StateSize) = xnew;
        Eigen::Map<Control>(su.data(), ControlSize) = u_noisy;
        update(sx, su, sx, su);
        return xnew;
    }
     inline State f_with_ref(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u,
                             const Eigen::Ref<const State> &x_ref, const Eigen::Ref<const Control> &u_ref)
    {
        static QVector<Scalar> sx(StateSize);
        static QVector<Scalar> su(ControlSize);
        static QVector<Scalar> sx_ref(StateSize);
        static QVector<Scalar> su_ref(ControlSize);

        Control u_noisy = u + cdist_.samples(1);

        State xnew = x + cp_.f(x, u_noisy) * dt_ + sdist_.samples(1);
        Eigen::Map<State>(sx.data(), StateSize) = xnew;
        Eigen::Map<Control>(su.data(), ControlSize) = u_noisy;
        Eigen::Map<State>(sx_ref.data(), StateSize) = x_ref;
        Eigen::Map<Control>(su_ref.data(), ControlSize) = u_ref;
        update(sx, su, sx_ref, su_ref);
        return xnew;
    }

signals:
    void update(const QVector<Scalar> &state, const QVector<Scalar> &control, const QVector<Scalar> &state_ref, const QVector<Scalar> &control_ref) const;

private:
Q_OBJECT
    Krang2D<Scalar> &cp_;
    Scalar dt_;
    Eigen::EigenMultivariateNormal<Scalar, StateSize> sdist_;
    Eigen::EigenMultivariateNormal<Scalar, ControlSize> cdist_;
};


#endif //DDP_KRANG2D_H
