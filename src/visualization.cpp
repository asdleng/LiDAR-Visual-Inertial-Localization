#include <visualization.h>
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose_visual;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
const Eigen::Vector3d CameraPoseVisualization::imlt = Eigen::Vector3d(-1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrt = Eigen::Vector3d( 1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imlb = Eigen::Vector3d(-1.0,  0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrb = Eigen::Vector3d( 1.0,  0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt0 = Eigen::Vector3d(-0.7, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt1 = Eigen::Vector3d(-0.7, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt2 = Eigen::Vector3d(-1.0, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::oc = Eigen::Vector3d(0.0, 0.0, 0.0);

void Eigen2Point(const Eigen::Vector3d& v, geometry_msgs::Point& p) {
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
}

CameraPoseVisualization::CameraPoseVisualization(float r, float g, float b, float a)
    : m_marker_ns("CameraPoseVisualization"), m_scale(0.2), m_line_width(0.01) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
}

void CameraPoseVisualization::setImageBoundaryColor(float r, float g, float b, float a) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
}

void CameraPoseVisualization::setOpticalCenterConnectorColor(float r, float g, float b, float a) {
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
}

void CameraPoseVisualization::setScale(double s) {
    m_scale = s;
}
void CameraPoseVisualization::setLineWidth(double width) {
    m_line_width = width;
}
void CameraPoseVisualization::add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.005;

    marker.color.g = 1.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point point0, point1;

    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    marker.points.push_back(point0);
    marker.points.push_back(point1);

    m_markers.push_back(marker);
}

void CameraPoseVisualization::add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.04;
    //marker.scale.x = 0.3;

    marker.color.r = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point point0, point1;

    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    marker.points.push_back(point0);
    marker.points.push_back(point1);

    m_markers.push_back(marker);
}


void CameraPoseVisualization::add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = m_line_width;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;


    geometry_msgs::Point pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1, pt_lt2;

    Eigen2Point(q * (m_scale *imlt) + p, pt_lt);
    Eigen2Point(q * (m_scale *imlb) + p, pt_lb);
    Eigen2Point(q * (m_scale *imrt) + p, pt_rt);
    Eigen2Point(q * (m_scale *imrb) + p, pt_rb);
    Eigen2Point(q * (m_scale *lt0 ) + p, pt_lt0);
    Eigen2Point(q * (m_scale *lt1 ) + p, pt_lt1);
    Eigen2Point(q * (m_scale *lt2 ) + p, pt_lt2);
    Eigen2Point(q * (m_scale *oc  ) + p, pt_oc);

    // image boundaries
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_lb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_rb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_rt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_lt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // top-left indicator
    marker.points.push_back(pt_lt0);
    marker.points.push_back(pt_lt1);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lt1);
    marker.points.push_back(pt_lt2);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // optical center connector
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);


    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    m_markers.push_back(marker);
}

void CameraPoseVisualization::reset() {
	m_markers.clear();
}

void CameraPoseVisualization::publish_by( ros::Publisher &pub, const std_msgs::Header &header ) {
	visualization_msgs::MarkerArray markerArray_msg;
	
	for(auto& marker : m_markers) {
		marker.header = header;
		markerArray_msg.markers.push_back(marker);
	}

	pub.publish(markerArray_msg);
}
void registerPub(ros::NodeHandle &n)
{
    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
}
void pubKeyPoses(const lvo::Map &map, const std_msgs::Header &header)
{
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "camera_init";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    geometry_msgs::Point pose_marker;
    Eigen::Vector3d correct_pose;
    cameraposevisual.reset();
    for(auto it = map.keyframes_.begin();it!=map.keyframes_.end();it++){
        correct_pose = it->get()->pos();
        Quaterniond R = Quaterniond(it->get()->T_f_w_.rotation_matrix().transpose());
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
        cameraposevisual.add_pose(it->get()->pos(), R);
        
    }
    pub_key_poses.publish(key_poses);
    cameraposevisual.publish_by(pub_camera_pose_visual, header);
}