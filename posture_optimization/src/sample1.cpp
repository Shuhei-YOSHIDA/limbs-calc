#include <ros/ros.h>
#include "sample_util.h"
#include <RBDynUrdf/Reader.h> //github repository:jorisv/RBDynUrdf
#include <RBDyn/FK.h>
#include "problem_define.h"
#include <random>

using namespace std;
using namespace rbdyn_urdf;
using namespace roboptim;

Urdf robotData;
sva::PTransformd X_O_T = sva::PTransformd(sva::RotY(M_PI/4), Vector3d(0.3, 0.1, 0.0));


bool sample1(rbd::MultiBody mb, rbd::MultiBodyConfig &mbcin, rbd::MultiBodyConfig &mbcout)
{
    //To fill some components
    mbcin.zero(mb);//set target posure as 0
    rbd::forwardKinematics(mb, mbcin);
    rbd::forwardVelocity(mb, mbcin);

    //BodyTask
    TaskPtr bodytask(new BodyTask(mb, "r_wrist", X_O_T));
    //PostureTask
    TaskPtr posturetask(new PostureTask(mb, mbcin));
    MultiTaskPtr tasks;
    tasks.push_back(pair<double, TaskPtr>(10000000, bodytask));
    tasks.push_back(pair<double, TaskPtr>(1.0, posturetask));

    //optimization
    typedef Solver<EigenMatrixDense> solver_t;

    //Create cost function
    boost::shared_ptr<F> f(new F(mb.nrDof(), mb));
    f->_tasks = tasks;

    //Create problem
    solver_t::problem_t pb(f);

    //Set bounds for all optimization parameters
    for (Function::size_type i = 0; i < pb.function().inputSize(); i++) {
        pb.argumentBounds()[i] = Function::makeInterval(-150.*M_PI/180., +150.*M_PI/180.);
        //Take care of type and order of joints such as Free, spherical and prism
        //and Inf can be used. 
    }

    //Set the starting point
    Function::vector_t start(pb.function().inputSize());
    //start = VectorXd(pb.function().inputSize());
    //start = Function::vector_t::Ones(pb.function().inputSize());
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_int_distribution<int> ranj(0,100);
    for (int i = 0; i < start.size(); i++) {
        start[i] = ranj(mt)/100.0;//set rundom value as start configuration
    }
    pb.startingPoint() = start;

    //Create constraints

    //Here we are relying on the CFSQP solver. 
    //You may change this string to load the solver you with to use:
    // - Ipopt: "ipopt", "ipopt-sparse", "ipopt-td"
    // - Eigen: "eigen-levenberg-marquardt"
    // etc.
    //The plugin is built for a given solver type, so choose it adequately
    SolverFactory<solver_t> factory("ipopt", pb);
    solver_t& solver = factory();

    //Compute the minimum and retrieve the result
    std::cout << "solver minimum " << std::endl;
    solver_t::result_t res = solver.minimum();

    //Display solver information
    cout << solver << endl;

    //Check in the minimization has succeeded
    //Process the result
    switch (res.which()) {
        case solver_t::SOLVER_VALUE:
        {
            //Get the result.
            Result& result = boost::get<Result>(res);
            f->xToMBC(result.x, mbcout);
            //Display the result
            cout << "A solution has been found: " << endl
                 << result << endl;
        }
        break;
        case solver_t::SOLVER_VALUE_WARNINGS:
        {
            //Get the result
            ResultWithWarnings& result = boost::get<ResultWithWarnings>(res);
            f->xToMBC(result.x, mbcout);
            //Display the result
            cout << "A solution with warnings has been found: " << endl
                 << result << endl;
        }
        break;
        case solver_t::SOLVER_NO_SOLUTION:
        {
            cout << "A solution should have been found. Failing ... " 
                 << endl << boost::get<SolverError>(res).what()
                 << endl;
            return false; 
        }
        break;
        case solver_t::SOLVER_ERROR:
        {
            cout << "SOLVER_ERROR occured. Failing ... " 
                 << endl << boost::get<SolverError>(res).what()
                 << endl;
            return false;
        }
        break;
    }

    //get results
    rbd::forwardKinematics(mb, mbcout);
    rbd::forwardVelocity(mb, mbcout);
    
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample1");
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


    bool res = sample1(mb, mbcin, mbcout);
    while(res && ros::ok()) {
        JointStateFromMBC(mb, mbcout, jmsg);
        MarkerArrayFromMBC(mb, mbcin, amsg);//show target posture
        visualization_msgs::Marker msg;
        MarkerSet(X_O_T, msg, amsg.markers[amsg.markers.size()-1].id+1);
        msg.header.stamp = amsg.markers[0].header.stamp;
        amsg.markers.push_back(msg);
        j_pub.publish(jmsg);
        a_pub.publish(amsg);

        ros::Duration(1.0).sleep();
    }
    cout << "end " << endl;
    
    return 0;
}
