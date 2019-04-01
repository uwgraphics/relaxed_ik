#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <math.h>
#include <chrono>
#include <boost/python/numpy.hpp>


using namespace std;
using namespace boost::python;
using namespace Eigen;
namespace np = boost::python::numpy;

typedef vector<double> vec;
typedef vector<vec> mat;

#define PI 3.14159265

class Arm {
public:
  int numDOF;
  boost::python::list displacements;
  boost::python::list axes;
  boost::python::list rotOffsets;
  boost::python::tuple dispOffset;
  boost::python::list velocity_limits;
  boost::python::list joint_limits;
  char* name;
  Matrix3d rotX;
  Matrix3d rotY;
  Matrix3d rotZ;


  Arm(boost::python::list axes, boost::python::list displacements,
    boost::python::list rotOffsets, boost::python::tuple dispOffset, char* name) {
    Py_Initialize();
    np::initialize();
    this->axes = axes;
    this->displacements = displacements;
    this->rotOffsets = rotOffsets;
    this->dispOffset = dispOffset;
    this->numDOF = len(displacements);
    this->name = name;

    this->rotX = MatrixXd::Zero(3,3);
    this->rotY = MatrixXd::Zero(3,3);
    this->rotZ = MatrixXd::Zero(3,3);

    this->rotX(0,0) = 1.0;
    this->rotY(1,1) = 1.0;
    this->rotZ(2,2) = 1.0;
  }

  Matrix3d& rot3(char axis, double s, double c) {
    if (axis == 'z' || axis == 'Z') {
      this->rotZ(0,0) = c;
      this->rotZ(0,1) = -s;
      this->rotZ(1,0) = s;
      this->rotZ(1,1) = c;
      return this->rotZ;
    }
    else if(axis == 'y' || axis == 'Y') {
      this->rotY(0,0) = c;
      this->rotY(0,2) = s;
      this->rotY(2,0) = -s;
      this->rotY(2,2) = c;
      return this->rotY;
    }
    else if(axis == 'x' || axis == 'X') {
      this->rotX(1,1) = c;
      this->rotX(1,2) = -s;
      this->rotX(2,1) = s;
      this->rotX(2,2) = c;
      return this->rotX;
    }
    else {
      cout << "ERROR: not a valid axis label";
    }
  }

  boost::python::list getFrames(boost::python::list state) {
    boost::python::list ret;
    boost::python::list pts;
    Vector3d pt = this->array(this->dispOffset);
    pts.append(this->tolist(pt));
    boost::python::list frames;
    Matrix3d rot = MatrixXd::Zero(3,3);
    rot(0,0) = 1.0;
    rot(1,1) = 1.0;
    rot(2,2) = 1.0;
    frames.append(np::array(this->tolist(rot)));

    for(int i = 0; i < this->numDOF; i++) {
      double s = sin(extract<double>(state[i]));
      double c = cos(extract<double>(state[i]));

      //TODO: do rotation offsets

      char curr_axis = extract<char>(this->axes[i]);

      Matrix3d rmat = this->rot3(curr_axis,s,c);
      rot = rot*rmat;
      Vector3d disp;
      disp(0) = extract<double>((this->displacements[i])[0]);
      disp(1) = extract<double>((this->displacements[i])[1]);
      disp(2) = extract<double>((this->displacements[i])[2]);
      // cout << rot(1,1);
      // cout << "\n";
      // high_resolution_clock::time_point t1 = high_resolution_clock::now();
      pt = (rot*disp) + pt;
      // high_resolution_clock::time_point t2 = high_resolution_clock::now();
      // duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
      // std::cout << "It took me " << time_span.count() << " seconds.\n";

      pts.append(np::array(this->tolist(pt)));
      frames.append(np::array(this->tolist(rot)));
    }

    ret.append(pts);
    ret.append(frames);
    return ret;
  }

private:

  Vector3d array(boost::python::list input) {
    Vector3d v;
    v(0) = extract<double>(input[0]);
    v(1) = extract<double>(input[1]);
    v(2) = extract<double>(input[2]);
    return v;
  }

  Vector3d array(boost::python::tuple input) {
    Vector3d v;
    v(0) = extract<double>(input[0]);
    v(1) = extract<double>(input[1]);
    v(2) = extract<double>(input[2]);
    return v;
  }

  boost::python::list tolist(Vector3d v) {
    boost::python::list l;
    l.append(v(0));
    l.append(v(1));
    l.append(v(2));
    return l;
  }

  mat tomat(Vector3d v) {
    mat ret;
    vec ve;
    ve.push_back(v(0));
    ve.push_back(v(1));
    ve.push_back(v(2));
    ret.push_back(ve);
    return ret;
  }

  mat tomat(Matrix3d m) {
    mat ret;
    vec row1;
    vec row2;
    vec row3;

    row1.push_back(m(0,0));
    row1.push_back(m(0,1));
    row1.push_back(m(0,2));

    row2.push_back(m(1,0));
    row2.push_back(m(1,1));
    row2.push_back(m(1,2));

    row3.push_back(m(2,0));
    row3.push_back(m(2,1));
    row3.push_back(m(2,2));

    ret.push_back(row1);
    ret.push_back(row2);
    ret.push_back(row3);

    return ret;
  }

  boost::python::list tolist(Matrix3d m) {
    boost::python::list ret;
    boost::python::list row1;
    boost::python::list row2;
    boost::python::list row3;

    row1.append(m(0,0));
    row1.append(m(0,1));
    row1.append(m(0,2));

    row2.append(m(1,0));
    row2.append(m(1,1));
    row2.append(m(1,2));

    row3.append(m(2,0));
    row3.append(m(2,1));
    row3.append(m(2,2));

    ret.append(row1);
    ret.append(row2);
    ret.append(row3);

    return ret;
  }

  np::ndarray convert_to_numpy(mat const & input) {
    u_int n_rows = input.size();
    u_int n_cols = input[0].size();
    boost::python::tuple shape = boost::python::make_tuple(n_rows, n_cols);
    boost::python::tuple stride = boost::python::make_tuple(sizeof(double));
    np::dtype dtype = np::dtype::get_builtin<double>();
    boost::python::object own;
    np::ndarray converted = np::zeros(shape, dtype);

    for (u_int i = 0; i < n_rows; i++)
    {
        shape = boost::python::make_tuple(n_cols);
        converted[i] = np::from_data(input[i].data(), dtype, shape, stride, own);
    }
    return converted;
  }

};


BOOST_PYTHON_MODULE(Arm_ext) {
    class_<Arm>("Arm", init<boost::python::list, boost::python::list,
    boost::python::list, boost::python::tuple, char*>())
        .def("getFrames", &Arm::getFrames)
        .def_readwrite("numDOF", &Arm::numDOF)
        .def_readwrite("name", &Arm::name)
        .def_readwrite("velocity_limits", &Arm::velocity_limits)
        .def_readwrite("rotOffsets", &Arm::rotOffsets)
        .def_readwrite("dispOffset", &Arm::dispOffset)
        .def_readwrite("joint_limits", &Arm::joint_limits)
        .def_readwrite("displacements", &Arm::displacements)
        .def_readwrite("axes", &Arm::axes)
    ;
}
