#include "krang2d.h"
#include "krang2dwindow.h"
#include <ddp/costs.hpp>
#include <ddp/ddp.hpp>
#include <ddp/mpc.hpp>
#include <ddp/util.hpp>
#include <QApplication>
#include <QtConcurrent/QtConcurrent>
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
    using Scalar = double;
    using Dynamics = Krang2D<Scalar>;
    using DDP_Opt = optimizer::DDP<Dynamics>;
    using Plant = Krang2DPlant;
    using Cost = Krang2DCost<Scalar>;
    using TerminalCost = Krang2DTerminalCost<Scalar>;
    using StateTrajectory = typename Dynamics::StateTrajectory ;
    using ControlTrajectory= typename Dynamics::ControlTrajectory ;

    // Log output
    util::DefaultLogger logger;
    bool verbose = true;

    // Timing
    Scalar tf = 20;
    Scalar dt = 0.01;
    auto time_steps = util::time_steps(tf, dt);
    int max_iterations = 15;

    // Dynamics
    Dynamics cp_dynamics(9.80, 115.25, 0.5, 1.8, 0.25, 0.005, 94.3);


    // Noisy Plant
    Scalar ssigma = 0.00;
    Scalar csigma = 0.00;
    Plant cp_plant(cp_dynamics, dt, ssigma, csigma);

    // Initial state th, dth, x, dx, desired state, initial control sequence
    Dynamics::State x0 = Dynamics::State::Zero();
    Dynamics::State xf; xf << 0.0, 0.0, 5, 0.0;
    Dynamics::ControlTrajectory u = Dynamics::ControlTrajectory::Zero(time_steps);

    // Costs
    Cost::StateHessian Q;
    Q.setZero();
    Q.diagonal() << 1, 0.1, 0.1, 0.1;

    Cost::ControlHessian R;
    R << 0.01;

    TerminalCost::Hessian Qf;
    Qf.setZero();
    Qf.diagonal() << 10000, 10000, 10000.0, 10000.0;

    Cost cp_cost(xf, Q, R);
    TerminalCost cp_terminal_cost(xf, Qf);

    // initialize DDP for trajectory planning
    DDP_Opt trej_ddp (dt, time_steps, max_iterations, &logger, verbose);

    // Get initial trajectory from DDP
    OptimizerResult<Dynamics> traj_results = trej_ddp.run(x0, u, cp_dynamics, cp_cost, cp_terminal_cost);

    StateTrajectory xs = traj_results.state_trajectory;
    ControlTrajectory us = traj_results.control_trajectory;

    logger.info("Obtained initial state trajectory");
    for (int m = 0; m < xs.cols(); ++m) {
        logger.info("\n");
        for (int n = 0; n < xs.rows(); ++n) {
            logger.info("%f ", xs(n, m));
        }
    }

    logger.info("Obtained initial control trajectory");
    for (int m = 0; m < us.cols(); ++m) {
        logger.info("\n");
        for (int n = 0; n < us.rows(); ++n) {
            logger.info("%f ", us(n, m));
        }
    }
    // Connect signals and configure plot window
    QApplication a(argc, argv);
    // final angle, then position
    MainWindow win(xf(0), xf(2), dt);
    QObject::connect(&cp_plant, &Plant::update, &win, &MainWindow::update, Qt::QueuedConnection);
    win.show();

    // Create New Cost Function

    // Create New Terminal Cost Function

    // Create new MPC

    using Result = ModelPredictiveController<Dynamics, optimizer::DDP>::Result;
    using StateRef = Eigen::Ref<const Dynamics::State>;


    // Define a termination condition (when no more threads alive)
    auto termination =
            [&](int i, Result &r, const StateRef &x, Cost &rc, TerminalCost &tc, Scalar true_cost)
            {
                return (QThreadPool::globalInstance()->activeThreadCount() == 0);
            };


    // ORIGINAL MPC

    // Initialize receding horizon controller
    // ModelPredictiveController<Dynamics, optimizer::DDP> mpc(dt, time_steps, max_iterations, &logger, verbose, /*DDP options*/ &logger, verbose);

    // Run in receding horizon mode
    // QtConcurrent::run([&](){ mpc.run(x0, u, termination, cp_dynamics, cp_plant, cp_cost, cp_terminal_cost); });

    // END ORIGINAL MPC

    // Run in receding horizon mode
    QtConcurrent::run([&](){

        int horizon = 10;
        Dynamics::State x = x0;
        Dynamics::State x_old = x0;

        // For Each Time Step
        for (int k = 0; k < (time_steps - 1); ++k) {
            int look_ahead;
            if ((time_steps - k) > horizon) {
               look_ahead = horizon;
            } else {
                look_ahead = (time_steps - 1) - k;
            }

            logger.info("\n");
            logger.info("iteration: %d", k);
            logger.info("look ahead: %d", look_ahead);

            // New Goal State
            Dynamics::State x_target = xs.col(k + look_ahead);

            Dynamics::ControlTrajectory u_horizon = Dynamics::ControlTrajectory::Zero(look_ahead);
            Dynamics::StateTrajectory x_horizon_ref = xs.block(0, k, 4, look_ahead);

//            logger.info("Obtained horizon state trajectory from ddp: ");
//            for(int m = 0; m < x_horizon_ref.cols(); ++m) {
//                logger.info("\n");
//                for (int n = 0; n < x_horizon_ref.rows(); ++n) {
//                    logger.info("%f ", x_horizon_ref(n, m));
//                }
//            }
//
//            logger.info("Obtained horizon control trajectory from ddp: ");
//            for(int m = 0; m <u_horizon.cols(); ++m) {
//                logger.info("\n");
//                for (int n = 0; n < u_horizon.rows(); ++n) {
//                    logger.info("%f ", u_horizon(n, m));
//                }
//            }
            // initialize DDP for trajectory planning
            DDP_Opt ddp_horizon (dt, look_ahead, max_iterations, &logger, verbose);

            Cost::StateHessian Q_mpc;
            Q_mpc.setZero();
            Q_mpc.diagonal() << 10, 0.1, 10, 0.1;
            Cost cp_cost_horizon(x_target, Q_mpc, R);
            TerminalCost cp_terminal_cost_horizon(x_target, Qf);

//
//            logger.info("\n");
//            logger.info("current state");
//            for (int n=0; n < 4; ++n) {
//                logger.info("\n");
//                logger.info("%f", x(n));
//                logger.info("\n");
//            }

//            logger.info("\n");
//            logger.info("new target");
//            for (int m =0; m < 4; ++m) {
//                logger.info("\n");
//                logger.info("%f", x_target(m));
//                logger.info("\n");
//            }

            OptimizerResult<Dynamics> results_horizon;
            results_horizon.control_trajectory = u_horizon;
            // Get initial trajectory from DDP

            results_horizon = ddp_horizon.run_horizon(x_old, u_horizon, x_horizon_ref, cp_dynamics, cp_cost_horizon, cp_terminal_cost_horizon);
//            results_horizon = ddp_horizon.run(x_old, u_horizon, cp_dynamics, cp_cost_horizon, cp_terminal_cost_horizon);

            Dynamics::Control u_new = results_horizon.control_trajectory.col(0);
            Dynamics::StateTrajectory x_horizon = results_horizon.state_trajectory;

//            logger.info("Obtained horizon state trajectory from optimizer: ");
//            for(int m = 0; m < xs_horizon.cols(); ++m) {
//                logger.info("\n");
//                for (int n = 0; n < xs_horizon.rows(); ++n) {
//                    logger.info("%f ", xs_horizon(n, m));
//                }
//            }
//            logger.info("\n");
//
//            logger.info("\n Obtained control trajectory from optimizer: \n");
//            for(int k =0; k < u_new.cols(); ++k) {
//                logger.info("%f", u_new(k));
//                logger.info("\n");
//            }
//
//
//            logger.info("Obtained control from optimizer: ");
//            for(int m = 0; m < u_new.rows(); ++m) { logger.info("%f ", u_new(m)); }
//            logger.info("\n");

            logger.info("new control %f", u_new(0));
            logger.info("reference state %f", xs(2, k));
            x = cp_plant.f_with_ref(x_old, u_new, xs.col(k), us.col(k));
            x_old = x;
        }

    });

    return a.exec();
}
