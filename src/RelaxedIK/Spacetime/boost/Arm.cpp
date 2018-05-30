#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace std;
using namespace boost::python;
using namespace Eigen;


class Arm {
public:
  int numDOF;
  list displacements;
  list axes;
  list rotOffsets;
  tuple dispOffset;
  list velocity_limits;
  list joint_limits;

  Arm(list axes, list displacements, list rotOffsets, tuple dispOffset) {
    this->axes = axes;
    this->displacements = displacements;
    this->rotOffsets = rotOffsets;
    this->dispOffset = dispOffset;
    numDOF = len(displacements);
  }

  list getFrames(list state) {
    list ret;
    list pts;
    pts.append(this->dispOffset);
    list frames;
    Matrix3f rot;
    rot(0,0) = 1.0;
    rot(1,1) = 1.0;
    rot(2,2) = 1.0;
    frames.append(this->tolist(rot));

    ret.append(pts);
    ret.append(frames);
    return ret;
  }


private:
  Vector3f array(list input) {
    Vector3f v;
    v(0) = extract<float>(input[0]);
    v(1) = extract<float>(input[1]);
    v(2) = extract<float>(input[2]);
    return v;
  }

  Vector3f array(tuple input) {
    Vector3f v;
    v(0) = extract<float>(input[0]);
    v(1) = extract<float>(input[1]);
    v(2) = extract<float>(input[2]);
    return v;
  }

  list tolist(Vector3f v) {
    list l;
    l.append(v(0));
    l.append(v(1));
    l.append(v(2));
    return l;
  }

  list tolist(Matrix3f m) {
    list ret;
    list row1;
    list row2;
    list row3;

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
};

BOOST_PYTHON_MODULE(Arm_ext) {
    class_<Arm>("Arm", init<list, list, list, tuple>())
        .def("getFrames", &Arm::getFrames)
        .def_readwrite("numDOF", &Arm::numDOF)
        .def_readwrite("velocity_limits", &Arm::velocity_limits)
        .def_readwrite("rotOffsets", &Arm::rotOffsets)
        .def_readwrite("dispOffset", &Arm::dispOffset)
        .def_readwrite("joint_limits", &Arm::joint_limits)
        .def_readwrite("displacements", &Arm::displacements)
        .def_readwrite("axes", &Arm::axes)

    ;
}
