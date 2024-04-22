/*
 * @Author: asdleng lengjianghao2006@163.com
 * @Date: 2023-02-25 11:25:46
 * @LastEditors: asdleng lengjianghao2006@163.com
 * @LastEditTime: 2023-09-02 22:18:40
 * @FilePath: /vio_in_lidar_map/src/vio_in_lidar_map/include/vio_in_lidar_map.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef VIO_IN_LIDAR_H_
#define VIO_IN_LIDAR_H_
#define FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))
#include <omp.h>
#include <common_lib.h>
#include <vikit/pinhole_camera.h>
#include <frame.h>
#include <map.h>
#include <feature.h>
#include <point.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <ikd-Tree/ikd_Tree.h>
#include <use-ikfom.hpp>
#include <so3_math.h>
#include <lsdlines.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <LineDetection3D.h>
#include "utils.h"
#include "line.h"
//#include <photometric_factor.h>
//#include "pose_local_parameterization.h"

namespace lvo{

class violm{
    public:
    double trans_thre = 0.1;
    double rot_thre = PI_M/12;
    bool ncc_en = false;
    int depth_search_radius = 4;
    double ncc_thre = 0.0;
    bool parallax_status = true;
    int down_sample_manner = 0;
    int down_sample_num = 50;
    bool update_key_frame_type = 1;
    BoxPointType LocalMap_Points; // 用于计算Box大小
    bool Localmap_Initialized = false;
    bool enable_triangulate;
    int effct_feat_num;
    float* res_last;
    bool* point_selected_surf;
    cv::Mat img_last, img_cur;
    int iterate_num_P,iterate_num_T;
    double skip_depth;
    double lightness_factor;
    vector<int> failed_index;
    vector<cv::Point2f> pts_last, pts_cur;
    vector<cv::Point2f> tracked_points;
    vector<int> tracked_start_index;
    vector<vector<cv::Point2f>> all_tracked_points;
    vector<V3D> ptws_last;
    vector<Sophus::SE3> pts_T0;
    vector<Sophus::SE3> pts_T;
    vector<vector<Sophus::SE3>> all_pts_T;
    vector<bool> is_tran;
    M3D tracked_R;
    V3D tracked_t;
    bool need_down_size = true;
    double max_blind = 100;
    bool map_is_based_on_LiDAR = false;
    std::string lines_file;
    double blind = 0;
    bool left_coord = false;
    bool equalize = false;
    bool cut_points_above_ground;
    bool enable_line;
    double img_scale = 1;
    int window_size = 10;
    int max_iteration = 10;
    std::vector<line2d> lines_2d;
    std::vector<line3d> lines_3d;
    std::vector<pairsmatch> matches2d3d;
    M3D JL_dR, JL_dt;
    MatrixXd JL;
    VectorXd resL;
    MatrixXd JL_sub;
    VectorXd resL_sub;
    
    M3D JP_dR, JP_dt;
    MatrixXd JP;
    VectorXd resP;
    MatrixXd JP_sub;
    VectorXd resP_sub;
    
    M3D JT_dR, JT_dt;
    MatrixXd JT;
    VectorXd resT;
    MatrixXd JT_sub;
    VectorXd resT_sub;

    double lamda = 0.0394;  // rad, 2度
    double line_threshold = 50;
    bool read_enable = true;
    bool b_viz_curv = false;
    bool b_voxel_filter = false;
    bool b_normalize_curv = true;

    const double scanPeriod = 0.1;

    const int systemDelay = 0;
    int systemInitCount = 0;
    bool systemInited = false;
    int N_SCANS = 0;
    int remove_frame_num=-1;
    float cloudCurvature[400000];
    int cloudSortInd[400000];
    int cloudNeighborPicked[400000]; // not picked 0, picked 1(不能作为特征点)
    int cloudLabel[400000];  

    bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }
    std::vector<ros::Publisher> pubEachScan;
    bool b_PUB_EACH_LINE = false;
    double MINIMUM_RANGE = 0.1;
    double THRESHOLD_FLAT = 0.01;
    double THRESHOLD_SHARP = 0.01;
    int kNumCurvSize = 5;
    int kNumRegion = 50;       // 6
    int kNumEdge = 2;          // 2 最大edge points数目
    int kNumFlat = 4;          // 4 最大 planar points数目
    int kNumEdgeNeighbor = 5;  // 5;
    int kNumFlatNeighbor = 5;  // 5;
    float kThresholdSharp = 50;          // 0.1;
    float kThresholdFlat = 30;           // 0.1;
    float kThresholdLessflat = 0.1;
    float kDistanceFaraway = 25;
    double linethre3d = 2.0;
    int linethre3d_proj = 10;
    int linethre2d = 20;
    std::unique_ptr<LineDetection3D> detector;
    
    std::vector<std::vector<cv::Point3d> > lines;
    
    std::deque<PointPtr> observed_points;
    M3D Rcl, Rci, Rli, Rcw, Ril, Rwc; // Rci: imu到camera,  Rcw：地面到camera
    V3D Pcl, Pci, Pli, Pcw, Pil, Pwc; // Pci:imu到camera，Pcw：地面到camera
    Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H,H_T_imgcovinv_H, lastG;
    Matrix<double, DIM_STATE, DIM_STATE> GL, HL_T_HL, lastGL;
    Matrix<double, DIM_STATE, DIM_STATE> GP, HP_T_HP, lastGP;
    Matrix<double, DIM_STATE, DIM_STATE> GT, HT_T_HT, lastGT;
    double fx,fy,cx,cy; // 相机参数
    double filter_size;
    int height, width;
    int grid_size, grid_n_width, grid_n_height,length;
    int patch_size, patch_size_total, patch_size_half;
    int iterate_num;
    int iterate_num_L;
    M3D Jdphi_dR, Jdp_dt, Jdp_dR; 
    KD_TREE<PointType> ikdtree;
    int pyr = 1;
    vk::AbstractCamera* cam;
    vk::PinholeCamera* pin_cam;
    pcl::VoxelGrid<PointType> downSizeFilter;
    PointCloudXYZI::Ptr sub_pcd_ptr;
    PointCloudXYZI::Ptr pcl_down;
    PointCloudXYZI::Ptr pcl_projected;
    PointCloudXYZI::Ptr pcl_line;
    PointCloudXYZI::Ptr triangulate_pts_world;
    PointCloudXYZI::Ptr triangulate_pts_body;
    PointCloudXYZI::Ptr normvec;
    PointCloudXYZI::Ptr laserCloudOri;
    PointCloudXYZI::Ptr corr_normvect;
    PointCloudXYZI::Ptr local_map;
    cv::Mat img_cp;
    cv::Mat img_depth;
    std::vector<V3D> adj_pts;
    cv::Mat img;
    M3D Rwi;
    V3D Pwi;
    double outlier_threshold = 300.0;
    Map map;
    bool enable_projection;
    std::list<std::tuple<Point *,int,int>> covisible_pair;
    MatrixXd J;
    VectorXd res;
    MatrixXd J_sub;
    MatrixXd img_cov;
    VectorXd res_sub;
    Frame* new_frame;
    double delta_dist_thre;
    float* patch_wrap;
    float* depth_img;
    int* outlier_map;
    std::vector<int> track_outlier;
    int max_ftr_list_num = 1;
    int grid_pts_num = 1;
    double** para_pose;
    int stage;
    bool need_keyframe;
    float *patch_cache; // 参考帧到当前帧
    int *pts_num_in_each_cell;
    double error;
    double last_error;
    bool first_frame=true;
    double score_threshold = 100.0;
    Matrix<double,2,6> jacobian_uv2pose;
    std::deque<lvo::Frame*> key_frames;

    long long frame_nums=0;
    double IMG_COV=100;
    double LINE_COV = 100;
    double POINT_COV = 100;
    double TRIAN_COV = 10;
    std::vector<int> trian_outlier;
    int body_pts_size;
    double solve_time;
    bool enable_BA = true;
    ofstream debug_file;
    ofstream lines_file_w;
    ifstream lines_file_i;
    ifstream exp_file_i;
    ifstream res_file_i;
    std::vector<double> index_response;
    std::vector<double> response;
    std::vector<double> exp_time;
    Eigen::Vector3d euler;  // 当前帧位姿
    enum Stage {
      STAGE_FIRST_FRAME,
      STAGE_DEFAULT_FRAME
    };
    struct Candidate {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Point* pt;       //!< 3D point.
      Vector2d px;     //!< projected 2D pixel location.
      float score;
      float edgescore;
      Candidate(Point* pt, Vector2d& px) : pt(pt), px(px) {};
      bool operator < (const Candidate& c2) const {
		    return score< c2.score;
      };
      bool operator > (const Candidate& c2) const {
		    return score> c2.score;
	    };
    };
    typedef std::list<Candidate > Cell;
    typedef std::vector<Cell*> CandidateGrid;
    struct Grid
    {
      CandidateGrid cells;
      vector<int> cell_order;
      int cell_size;
      int grid_n_cols;
      int grid_n_rows;
    };
    Grid grid;
    std::deque<double> weights;
    double theta_thre;
    double edgescore_threshold = 100.0;
    cv::Mat gradientX, gradientY;
    cv::Mat gradientMagnitude;
    StatesGroup* state;
    StatesGroup* state_propagat;
    std::vector<cv::Mat> pyramid;
    double current_pyr_error;
    double last_pyr_error;
    void Init(PointCloudXYZI::Ptr ptr);
    void updateFrameState();
    void setKF(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);
    void setState(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);
    void setStatePropagate(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);
    int obs_points();
    void set_camera2lidar(vector<double>& R,  vector<double>& P);
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void reset_grid();
    bool CalculateJandRes(int level);
    bool CalculateJLandResL();
    bool CalculateJPandResP();
    bool CalculateJTandResT();
    void Process(const cv::Mat &img, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);
    double NCC(float* ref_patch, float* cur_patch, int patch_size);

    void projection();
    void addObservation();

    void findalignment(); // 找到曾经被观测到的点 
    void Updatekf(const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);
    void UpdateCamPose();
    
    void photometricBA();
    void getWarpMatrixAffine(
      const vk::AbstractCamera& cam,
      const Vector2d& px_ref,
      const Vector3d& f_ref,
      const double depth_ref,
      const Sophus::SE3& T_cur_ref,
      const int level_ref,    // px_ref对应特征点的金字塔层级
      const int pyramid_level,
      const int halfpatch_size,
      Matrix2d& A_cur_ref);
    void warpAffine(
      const Matrix2d& A_cur_ref,
      const cv::Mat& img_ref,
      const Vector2d& px_ref,
      const int level_ref,
      const int search_level,
      const int pyramid_level,
      const int halfpatch_size,
      float* patch);

    bool depthContinue(const V2D &px, const double & depth);
    bool depthContinue2(const V2D &px, const double & depth,const V3D &ptw);
    bool featAdd();
    template <typename T>
    void reduceVector(vector<T> &v, vector<uchar> status);
    void reduceVector2(vector<V3D> &v, vector<uchar> status);
    void getpatch(cv::Mat img, V2D pc, float* patch_tmp, int level);
    void dpi(V3D p, MD(2,3)& J);
    void projectionConstraint(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);
    void lineConstraint(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);
    void projectionLine();
    void triangulate(esekfom::esekf<state_ikfom, 12, input_ikfom>& kf);
    void localmapGenerate();
    


    violm(){
      local_map.reset(new pcl::PointCloud<PointType>());
      sub_pcd_ptr.reset(new pcl::PointCloud<PointType>());
      pcl_down.reset(new pcl::PointCloud<PointType>());
      pcl_line.reset(new pcl::PointCloud<PointType>());
      pcl_projected.reset(new pcl::PointCloud<PointType>());
      triangulate_pts_world.reset(new pcl::PointCloud<PointType>());
      triangulate_pts_body.reset(new pcl::PointCloud<PointType>());
      normvec.reset(new pcl::PointCloud<PointType>(100000,1));
      laserCloudOri.reset(new pcl::PointCloud<PointType>(100000,1));
      corr_normvect.reset(new pcl::PointCloud<PointType>(100000,1));
      detector = std::make_unique<LineDetection3D>();
    };
    ~violm(){};
};










}
#endif // VIO_IN_LIDAR_H_