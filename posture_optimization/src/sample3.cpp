// Inverse kinematics as convex problem
#include <ros/ros.h>
#include "sample_util.h"
#include <RBDynUrdf/Reader.h> //github repository:jorisv/RBDynUrdf
#include <RBDyn/FK.h>
#include <RBDyn/EulerIntegration.h>

#include "ik_problem_define.h"
#include "capsule_set.h"
#include "sch_set.h"
#include <chrono>

using namespace std;
using namespace rbdyn_urdf;
using namespace roboptim;

Urdf robotData;
typedef Solver<EigenMatrixDense> solver_t;
boost::shared_ptr<F_ik> f_ik;
boost::shared_ptr<G_q_vd_up> g_q_vd_up;
boost::shared_ptr<G_q_vd_lo> g_q_vd_lo;
boost::shared_ptr<solver_t::problem_t> pb_ptr;
boost::shared_ptr<SolverFactory<solver_t>> factory_ptr;
MultiTaskPtr tasks;

void setup(rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
{
    sva::PTransformd X_O_T_tmp = sva::PTransformd::Identity();
    //sva::PTransformd X_O_T_tmp(Vector3d(0.4, -0.2, 0.1));
    TaskPtr bodytask(new BodyTask(mb, "r_wrist", X_O_T_tmp));
    tasks.push_back(pair<double, TaskPtr>(1, bodytask));

    f_ik = boost::shared_ptr<F_ik>(new F_ik(mb.nrDof(), mb, mbc));
    f_ik->_tasks = tasks;

    g_q_vd_up = boost::shared_ptr<G_q_vd_up>(new G_q_vd_up(mb.nrDof(), mb, mbc));
    g_q_vd_lo = boost::shared_ptr<G_q_vd_lo>(new G_q_vd_lo(mb.nrDof(), mb, mbc));

    pb_ptr = boost::shared_ptr<solver_t::problem_t>(new solver_t::problem_t(f_ik));

    F_ik::intervals_t bounds;
    solver_t::problem_t::scaling_t scaling;

    // Add constraints
    // upper
    for (int i = 0; i < mb.nrDof(); i++) {
        bounds.push_back(Function::makeUpperInterval(0.));
        scaling.push_back(1.);
    }
    pb_ptr->addConstraint(
            boost::static_pointer_cast<DifferentiableFunction>(g_q_vd_up),
            bounds, scaling);
    bounds.clear();
    scaling.clear();

    // lower
    for (int i = 0; i < mb.nrDof(); i++) {
        bounds.push_back(Function::makeLowerInterval(0.));
        scaling.push_back(1.);
    }
    pb_ptr->addConstraint(
            boost::static_pointer_cast<DifferentiableFunction>(g_q_vd_lo),
            bounds, scaling);
    bounds.clear();
    scaling.clear();

    //Set bounds for all optimization parameters
    //for (Function::size_type i = 0; i < pb_ptr->function().inputSize(); i++) {
    //    pb_ptr->argumentBounds()[i] = Function::makeInterval(-7.*M_PI/180., +7.*M_PI/180.);
    //}

    Function::vector_t start(pb_ptr->function().inputSize());
    for (int i = 0; i < start.size(); i++) {
        start[i] = 0;
    }
    pb_ptr->startingPoint() = start;


    factory_ptr = boost::shared_ptr<SolverFactory<solver_t>>(
        new SolverFactory<solver_t>("ipopt", *pb_ptr));
}

// call it at each iterate
bool sample3(rbd::MultiBody mb,
             rbd::MultiBodyConfig mbcin, rbd::MultiBodyConfig &mbcout,
             sensor_msgs::JointState &jmsg)
{
    rbd::forwardKinematics(mb, mbcin);
    rbd::forwardVelocity(mb, mbcin);
    f_ik->_mbc = mbcin;
    g_q_vd_up->_mbc = mbcin;
    g_q_vd_lo->_mbc = mbcin;

    solver_t& solver = (*factory_ptr)();
    //solver.parameters()["ipopt.tol"].value = double(1e-3);
    solver.parameters()["ipopt.linear_solver"].value = std::string("ma57");
    solver.parameters()["ipopt.mu_strategy"].value = std::string("adaptive");
    solver.reset();
    solver_t::result_t res = solver.minimum();

    //Check in the minimization has succeeded
    //Process the result
    rbd::MultiBodyConfig tmpmbc(mb);
    tmpmbc = mbcin;
    switch (res.which()) {
        case solver_t::SOLVER_VALUE:
        {
            //Get the result.
            Result& result = boost::get<Result>(res);
            tmpmbc.alpha = rbd::vectorToDof(mb, result.x);
            //std::cout << "constraint " << result.constraints << std::endl;
            std::cout << solver << std::endl;
        }
        break;
        case solver_t::SOLVER_VALUE_WARNINGS:
        {
            //Get the result
            ResultWithWarnings& result = boost::get<ResultWithWarnings>(res);
            tmpmbc.alpha = rbd::vectorToDof(mb, result.x);
        }
        break;
        default:
        {
            std::cout << "no solution" << std::endl;
            return false;
        }
        break;
    }

    //std::cout << "alpha " << tmpmbc.alpha << std::endl;
    //std::cout << "q " << tmpmbc.q << std::endl;
    rbd::eulerIntegration(mb, tmpmbc, 1);
    rbd::forwardKinematics(mb, tmpmbc);
    mbcout = tmpmbc;
    sensor_msgs::JointState msg;
    JointStateFromMBC(mb, tmpmbc, msg);
    jmsg = msg;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample3");
    ros::NodeHandle n;

    string urdf_string;
    if (n.getParam("/robot_description", urdf_string)) {
        robotData = readUrdf(urdf_string);
    }
    else {
        cerr << "urdf was not loaded from parameter " << endl;
        return -1;
    }

    sensor_msgs::JointState jmsg;
    visualization_msgs::MarkerArray amsg;

    ros::Publisher j_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher a_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1);

    rbd::MultiBody mb = robotData.mbg.makeMultiBody("base_link", rbd::Joint::Fixed);
    rbd::MultiBodyConfig mbcin(mb);
    rbd::MultiBodyConfig mbcout(mb);
    mbcin.zero(mb);
    rbd::forwardKinematics(mb, mbcin);
    rbd::forwardVelocity(mb, mbcin);

    // initialize roboptims
    setup(mb, mbcin);

    // 100Hz Inverse kinematics
    //sva::PTransformd xot_tmp = sva::PTransformd(sva::RotY(M_PI/4), Vector3d(0.4, -0.4, 0.0));
    //sva::PTransformd xot_tmp = sva::PTransformd(Vector3d(0.4, -0.2, 0.05));
    sva::PTransformd xot_tmp = sva::PTransformd(Vector3d(0.3, -0.3, 0.2));
    visualization_msgs::MarkerArray marray;
    visualization_msgs::Marker mkr;
    MarkerSet(xot_tmp, mkr, 0);
    marray.markers.push_back(mkr);

    TaskPtr t_ptr(new BodyTask(mb, "r_wrist", xot_tmp));
    f_ik->_tasks[0].second = t_ptr;
    while (ros::ok()) {
        dynamic_pointer_cast<BodyTask>(t_ptr)->_X_O_T = xot_tmp;
        auto start = std::chrono::system_clock::now();
        bool res = sample3(mb, mbcin, mbcout, jmsg); //iterate inverse kinematics
        auto end = std::chrono::system_clock::now();
        auto diff = end - start;
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(diff).count()
                  << " ms is elapsed for optimization iterated" << std::endl;
        if (res) {
            mbcin = mbcout;
            auto stamp = ros::Time::now();
            jmsg.header.stamp = stamp;
            j_pub.publish(jmsg);
        }
        else std::cout << "no res " << std::endl;

        for (auto&& var : marray.markers) {
            var.header.stamp = ros::Time::now();
        }
        a_pub.publish(marray);

        ros::Duration(0.010).sleep();
    }


    return 0;
}
