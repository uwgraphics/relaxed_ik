#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <string>
#include <math.h>
#include <vector>

using namespace std;
namespace np = boost::python::numpy;
namespace p = boost::python;

int main(int argc, char **argv) {
  Py_Initialize();
  np::initialize();
}

string greet() {
  return "hello!";
}

double orientation_multiEE_obj(p::object frames, p::object goal_quats, p::list weights) {
    double num_ee = p::len(frames);

    double sum = 0.0;

    for(int i=0; i < num_ee; i++) {
        
    }
}

double position_multiEE_obj(p::object frames, p::object eeGoals, p::list weights) {
    double num_ee = p::len(frames);

    double sum = 0.0;

    for(int i = 0; i < num_ee; i++) {
        p::list f = p::extract<p::list>(frames[i]);

        p::list positions = p::extract<p::list>(f[0]);
        int num_jts = p::len(positions);

        np::ndarray eePos = p::extract<np::ndarray>(positions[num_jts-1]);

        double eX = p::extract<double>(eePos[0]);
        double eY = p::extract<double>(eePos[1]);
        double eZ = p::extract<double>(eePos[2]);

        np::ndarray eeGoal = p::extract<np::ndarray>(eeGoals[i]);
        double gX = p::extract<double>(eeGoal[0]);
        double gY = p::extract<double>(eeGoal[1]);
        double gZ = p::extract<double>(eeGoal[2]);

        double val = pow(eX-gX, 2.0) + pow(eY-gY, 2.0) + pow(eZ-gZ, 2.0);
        val = sqrt(val);
        sum += p::extract<double>(weights[i]) * val;
        // return p::extract<double>(eePos[0]);
    }

    return sum;
}

double min_jt_jerk_obj(np::ndarray x, p::object prev, p::object prev2, p::object prev3) {
    int vec_len = (int) p::len(x);

    double sum = 0.0;

    for(int i=0; i < vec_len; i++) {
        double v1 = p::extract<double>(x[i]);
        double v2 = p::extract<double>(prev[i]);
        double v3 = p::extract<double>(prev2[i]);
        double v4 = p::extract<double>(prev3[i]);

        double vel3 = v3 - v4;
        double vel2 = v2 - v3;
        double vel1 = v1 - v2;

        sum += pow((vel2 - vel3) - (vel1 - vel2), 2.0);
    }

    return sqrt(sum);
}

double min_jt_accel_obj(np::ndarray x, p::object prev, p::object prev2) {
    int vec_len = (int) p::len(x);

    double sum = 0.0;

    for(int i=0; i < vec_len; i++) {
        double v1 = p::extract<double>(x[i]);
        double v2 = p::extract<double>(prev[i]);
        double v3 = p::extract<double>(prev2[i]);

        sum += pow((v2 - v3) - (v1 - v2), 2.0);
    }

    return sqrt(sum);
}

double min_jt_vel_obj(np::ndarray x, p::object prev) {
    int vec_len = (int) p::len(x);

    double sum = 0.0;

    for(int i=0; i < vec_len; i++) {
        double v1 = p::extract<double>(x[i]);
        double v2 = p::extract<double>(prev[i]);

        sum += pow((v1-v2), 2.0);
    }
    return sqrt(sum);
}


double nloss(double x_val, double t, double d, double c, double f, double g) {
    return -exp( (- pow( (x_val - t), d) ) / (2.0 * pow(c,2.0))) + f * pow( (x_val - t), g);
}

BOOST_PYTHON_MODULE(objectives_ext)
{
    using namespace boost::python;
    def("greet", greet);
    def("orientation_multiEE_obj",orientation_multiEE_obj);
    def("position_multiEE_obj", position_multiEE_obj);
    def("min_jt_vel_obj", min_jt_vel_obj);
    def("min_jt_accel_obj", min_jt_accel_obj);
    def("min_jt_jerk_obj", min_jt_jerk_obj);
    def("nloss", nloss);
}
