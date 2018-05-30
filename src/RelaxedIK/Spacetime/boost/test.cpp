#include <boost/python.hpp>

boost::python::list greet()
{
  boost::python::list l;
  l.append(3.0);
  l.append(2.0);
  l.append(1.0);
  return l;
}

BOOST_PYTHON_MODULE(hello_ext)
{
    using namespace boost::python;
    def("greet", greet);
}
