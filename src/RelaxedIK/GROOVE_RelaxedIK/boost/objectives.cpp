// author: Danny Rakita

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <string>
#include <math.h>
#include <vector>
#include <Eigen/Dense>

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

inline double SIGN(double x) {
	return (x >= 0.0f) ? +1.0 : -1.0;
}

inline double NORM(double a, double b, double c, double d) {
	return sqrt(a * a + b * b + c * c + d * d);
}
// quaternion = [w, x, y, z]'
vector<double> mRot2Quat(np::ndarray m) {
	double r11 = p::extract<double>(m[0][0]);
	double r12 = p::extract<double>(m[0][1]);
	double r13 = p::extract<double>(m[0][2]);
	double r21 = p::extract<double>(m[1][0]);
	double r22 = p::extract<double>(m[1][1]);
	double r23 = p::extract<double>(m[1][2]);
	double r31 = p::extract<double>(m[2][0]);
	double r32 = p::extract<double>(m[2][1]);
	double r33 = p::extract<double>(m[2][2]);
	double q0 = (r11 + r22 + r33 + 1.0) / 4.0;
	double q1 = (r11 - r22 - r33 + 1.0) / 4.0;
	double q2 = (-r11 + r22 - r33 + 1.0) / 4.0;
	double q3 = (-r11 - r22 + r33 + 1.0) / 4.0;
	if (q0 < 0.0) {
		q0 = 0.0;
	}
	if (q1 < 0.0) {
		q1 = 0.0;
	}
	if (q2 < 0.0) {
		q2 = 0.0;
	}
	if (q3 < 0.0) {
		q3 = 0.0;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0;
		q1 *= SIGN(r32 - r23);
		q2 *= SIGN(r13 - r31);
		q3 *= SIGN(r21 - r12);
	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q0 *= SIGN(r32 - r23);
		q1 *= +1.0;
		q2 *= SIGN(r21 + r12);
		q3 *= SIGN(r13 + r31);
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q0 *= SIGN(r13 - r31);
		q1 *= SIGN(r21 + r12);
		q2 *= +1.0;
		q3 *= SIGN(r32 + r23);
	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q0 *= SIGN(r21 - r12);
		q1 *= SIGN(r31 + r13);
		q2 *= SIGN(r32 + r23);
		q3 *= +1.0;
	}
	else {
		printf("coding error\n");
	}
	double r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;

	vector<double> res(4);
	res[0] = q0; res[1] = q1; res[2] = q2; res[3] = q3;

	return res;
}

vector<double> quaternion_multiply(vector<double> q1, vector<double> q0) {
    vector<double> q(4);
    q[0] = -q1[1]*q0[1] - q1[2]*q0[2] - q1[3]*q0[3] + q1[0]*q0[0];
    q[1] =  q1[1]*q0[0] + q1[2]*q0[3] - q1[3]*q0[2] + q1[0]*q0[1];
    q[2] = -q1[1]*q0[3] + q1[2]*q0[0] + q1[3]*q0[1] + q1[0]*q0[2];
    q[3] =  q1[1]*q0[2] - q1[2]*q0[1] + q1[3]*q0[0] + q1[0]*q0[3];
    return q;
}

vector<double> quaternion_inverse(vector<double> q) {
    vector<double> q_i(4);
    q_i[0] = q[0];
    q_i[1] = -q[1];
    q_i[2] = -q[2];
    q_i[3] = -q[3];
    double dot = q_i[0]*q_i[0] + q_i[1]*q_i[1] + q_i[2]*q_i[2] + q_i[3]*q_i[3];
    q_i[0] = q_i[0] / dot;
    q_i[1] = q_i[1] / dot;
    q_i[2] = q_i[2] / dot;
    q_i[3] = q_i[3] / dot;
    return q_i;
}

vector<double> quaternion_log(vector<double> q) {
    vector<double> rot_vec(3);
    rot_vec[0] = q[1]; rot_vec[1] = q[2]; rot_vec[2] = q[3];
    if(abs(q[0]) < 1.0) {
        double a = acos(q[0]);
        double sina = sin(a);
        if (abs(sina) >= 0.05) {
           double c = a/sina;
           rot_vec[0] *= c;
           rot_vec[1] *= c;
           rot_vec[2] *= c;
        }
    }

    return rot_vec;

}

vector<double> quaternion_disp(vector<double> q1, vector<double> q2) {
    vector<double> inv = quaternion_inverse(q1);
    vector<double> m = quaternion_multiply(inv, q2);
    return quaternion_log(m);
}

double orientation_multiEE_obj(p::object frames, p::object goal_quats, p::list weights) {
    double num_ee = p::len(frames);

    double sum = 0.0;

    for(int q=0; q < num_ee; q++) {
        p::list f = p::extract<p::list>(frames[q]);

        p::list rot_mats = p::extract<p::list>(f[1]);
        int num_jts = p::len(rot_mats);

        np::ndarray ee_rot = p::extract<np::ndarray>(rot_mats[num_jts-1]);

        vector<double> ee_quat = mRot2Quat(ee_rot);
        vector<double> ee_quat2(4);
        ee_quat2[0] = -ee_quat[0];
        ee_quat2[1] = -ee_quat[1];
        ee_quat2[2] = -ee_quat[2];
        ee_quat2[3] = -ee_quat[3];

        p::object goal_quat_py = p::extract<p::object>(goal_quats[q]);
        vector<double> goal_quat(4);
        goal_quat[0] = p::extract<double>(goal_quat_py[0]);
        goal_quat[1] = p::extract<double>(goal_quat_py[1]);
        goal_quat[2] = p::extract<double>(goal_quat_py[2]);
        goal_quat[3] = p::extract<double>(goal_quat_py[3]);

        vector<double> r1 = quaternion_disp(goal_quat, ee_quat);
        vector<double> r2 = quaternion_disp(goal_quat, ee_quat2);

        double disp = sqrt(r1[0]*r1[0] + r1[1]*r1[1] + r1[2]*r1[2]);
        double disp2 = sqrt(r2[0]*r2[0] + r2[1]*r2[1] + r2[2]*r2[2]);

        sum += p::extract<double>(weights[q])*min(disp, disp2);

    }

    return sum;
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
