from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointField, PointCloud2
import rospy


fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]

pts = []
for i in xrange(50000):
    pts.append([0.0,0.0,0.0])

cloud = pc2.create_cloud(None, fields, pts)

rospy.init_node('test')

pub = rospy.Publisher('test', PointCloud2, queue_size=3)

while not rospy.is_shutdown():
    # cloud = pc2.create_cloud(None, fields, pts)
    pub.publish(cloud)
    x = pc2.read_points(cloud)
    print 'yep'

# x = pc2.read_points(cloud)

# for i in x:
#   print i