#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include "../tools/tools_logger.hpp"

using namespace std;

#define IS_VALID(a) ( ( abs( a ) > 1e8 ) ? true : false )
#define DEBUG

/*
 * pointxyzinormal: 包含除了点的x,y,z信息和密集程度信息之外，还有坐标(x,y,z)和曲率
 */
typedef pcl::PointXYZINormal PointType;

ros::Publisher pub_full, pub_surf, pub_corn;

enum LID_TYPE {//雷达类型
    MID,
    HORIZON,
    VELO16,
    OUST64
};

enum Feature {//特征点类型
    Nor,
    Poss_Plane, // 可能平面
    Real_Plane, // 平面
    Edge_Jump, // 跳跃边
    Edge_Plane, // 平面交接边
    Wire, //细线
    ZeroPoint // 接近0度（边）
};
enum Surround {
    Prev,
    Next
};
enum E_jump {
    Nr_nor,
    Nr_zero, // 接近0度
    Nr_180, // 接近180度
    Nr_inf, // 接近远端
    Nr_blind // 接近近端
};

struct orgtype { // 用于记录每个点的距离、角度、特征种类等属性
    double  range; // 平面距离
    double  dista; // 与后一个点的间距的平方
    double  angle[ 2 ]; // cos（当前点指向前一点或后一点的向量， ray）
    double  intersect; // 当前点与相邻两点的夹角cos值
    E_jump edj[2]; // 枚举类型
    Feature ftype; // 枚举类型
    orgtype() { // 构造函数
        range = 0; // 雷达点的深度信息
        edj[Prev] = Nr_nor; // 0
        edj[Next] = Nr_nor; // 0
        ftype = Nor;         // 0
        intersect = 2;
    }
};

const double rad2deg = 180 * M_1_PI;

int lidar_type;//雷达类型
double blind, inf_bound;
int N_SCANS;//扫描线数
int group_size;
double disA, disB;
double limit_maxmid, limit_midmin, limit_maxmin;
double p2l_ratio;
double jump_up_limit, jump_down_limit;
double cos160;
double edgea, edgeb;
double smallp_intersect, smallp_ratio;
int point_filter_num;//采样间隔
int g_if_using_raw_point = 1;
int g_LiDAR_sampling_point_step = 3;

int SCAN_RATE, time_unit;//雷达类型、采样间隔、扫描线数、扫描频率
bool feature_enabled, given_offset_time;// 是否提取特征、是否进行时间偏移

void mid_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

void horizon_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);

void velo16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

void give_feature(pcl::PointCloud <PointType> &pl, vector <orgtype> &types, pcl::PointCloud <PointType> &pl_corn,
                  pcl::PointCloud <PointType> &pl_surf);

void pub_func(pcl::PointCloud <PointType> &pl, ros::Publisher pub, const ros::Time &ct);

int plane_judge(const pcl::PointCloud <PointType> &pl, vector <orgtype> &types, uint i, uint &i_nex,
                Eigen::Vector3d &curr_direct);

bool small_plane(const pcl::PointCloud <PointType> &pl, vector <orgtype> &types, uint i_cur, uint &i_nex,
                 Eigen::Vector3d &curr_direct);

bool edge_jump_judge(const pcl::PointCloud <PointType> &pl, vector <orgtype> &types, uint i, Surround nor_dir);

int main(int argc, char **argv) {
    /**
     * (1)初始化名为"feature_extract"的ros节点
     */
    ros::init(argc, argv, "feature_extract");
    ros::NodeHandle n;

    /**
     * (2)从参数服务器上获取参数,如果获取失败则=默认值
     * @note NodeHandle::param(const std::string& param_name, T& param_val, const T& default_val)
     */
    n.param<int>("Lidar_front_end/lidar_type", lidar_type, 0);
    n.param<double>("Lidar_front_end/blind", blind, 0.1);
    n.param<double>("Lidar_front_end/inf_bound", inf_bound, 4);
    n.param<int>("Lidar_front_end/N_SCANS", N_SCANS, 1);
    n.param<int>("Lidar_front_end/group_size", group_size, 8);
    n.param<double>("Lidar_front_end/disA", disA, 0.01);
    n.param<double>("Lidar_front_end/disB", disB, 0.1);
    n.param<double>("Lidar_front_end/p2l_ratio", p2l_ratio, 225);
    n.param<double>("Lidar_front_end/limit_maxmid", limit_maxmid, 6.25);
    n.param<double>("Lidar_front_end/limit_midmin", limit_midmin, 6.25);
    n.param<double>("Lidar_front_end/limit_maxmin", limit_maxmin, 3.24);
    n.param<double>("Lidar_front_end/jump_up_limit", jump_up_limit, 170.0);
    n.param<double>("Lidar_front_end/jump_down_limit", jump_down_limit, 8.0);
    n.param<double>("Lidar_front_end/cos160", cos160, 160.0);
    n.param<double>("Lidar_front_end/edgea", edgea, 2);
    n.param<double>("Lidar_front_end/edgeb", edgeb, 0.1);
    n.param<double>("Lidar_front_end/smallp_intersect", smallp_intersect, 172.5);
    n.param<double>("Lidar_front_end/smallp_ratio", smallp_ratio, 1.2);
    n.param<int>("Lidar_front_end/point_filter_num", point_filter_num, 1);
    n.param<int>("Lidar_front_end/point_step", g_LiDAR_sampling_point_step, 3);
    n.param<int>("Lidar_front_end/using_raw_point", g_if_using_raw_point, 1);
///    n.param<bool>("feature_extract_enable", feature_enabled, false);// 是否提取特征点（FAST_LIO2默认不进行特征点提取）

    // 设置需要用到的全局变量 - cos(角度->弧度)
    jump_up_limit = cos(jump_up_limit / 180 * M_PI);// cos(170度)
    jump_down_limit = cos(jump_down_limit / 180 * M_PI);// cos(8度)
    cos160 = cos(cos160 / 180 * M_PI);// cos(160度)
    smallp_intersect = cos(smallp_intersect / 180 * M_PI);// cos(172.5度)

    ros::Subscriber sub_points;

    /**
    * (3)根据lidar_type的值进行lidar数据的订阅设置
    *      绑定订阅后的回调函数
    */
    switch (lidar_type) {
        case MID:// enum MID = 0 : 默认
            printf("MID40\n");
            // 订阅/livox/lidar原始数据,回调函数中转换数据(/laser_cloud),并提取角/面特征后发布出去(laser_cloud_sharp/laser_cloud_flat)
            // 允许指定hints到roscpp的传输层，这里使用没延迟的TCP。其实也可以使用UPD传输，
            sub_points = n.subscribe("/livox/lidar", 1000, mid_handler, ros::TransportHints().tcpNoDelay());
            break;

        case HORIZON:// enum HORIZON = 1 : r3live_bag_ouster.launch中设置
            printf("HORIZON\n");
            sub_points = n.subscribe("/livox/lidar", 1000, horizon_handler, ros::TransportHints().tcpNoDelay());
            break;

        case VELO16:// enum VELO16 = 2 : TODO
            printf("VELO16\n");
            sub_points = n.subscribe("/velodyne_points", 1000, velo16_handler, ros::TransportHints().tcpNoDelay());
            break;

        case OUST64:// enum OUST64 = 3 : TODO
            printf("OUST64\n");
            sub_points = n.subscribe("/os_cloud_node/points", 1000, oust64_handler, ros::TransportHints().tcpNoDelay());
            break;

        default:
            printf("Lidar type is wrong.\n");
            exit(0);
            break;
    }

    /**
     * (4)设置需要发布的消息格式
     *      回调函数接收到原始点云后,将其转换为指定格式,并提取角点特征和面特征
     *      然后通过pub_full,pub_surf,pub_corn将消息发布出去
     */
    pub_full = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 100);//发布转换后的消息
    pub_surf = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);//发布面特征消息
    pub_corn = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);//发布角点特征信息

    ros::spin();
    return 0;
}

/**
 * @note LiDAR_front_end的回调函数
 *      接收/livox/lidar原始数据,回调函数中转换数据(/laser_cloud),
 *      并提取角/面特征后发布出去(laser_cloud_sharp/laser_cloud_flat)
 */
double vx, vy, vz;

void mid_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // typedef pcl::PointXYZINormal PointType;
    pcl::PointCloud <PointType> pl;//存放lidar点的一维数组
    //转换为Pointcloud2类型
    pcl::fromROSMsg(*msg, pl);//通过PointCloud2格式的msg初始化PointXYZINormal的pl

    pcl::PointCloud <PointType> pl_corn, pl_surf;// 保存提取的角点和平面点
    vector <orgtype> types;// 自定义的结构体
    uint plsize = pl.size() - 1;//lidar点数量
    pl_corn.reserve(plsize);//预留size
    pl_surf.reserve(plsize);//预留size
    types.resize(plsize + 1);

    // 遍历msg中的前n-1个点,保存在types中(关键内容:range和dista)
    for (uint i = 0; i < plsize; i++) {
        types[i].range = pl[i].x;
        vx = pl[i].x - pl[i + 1].x;// 相邻两点做差
        vy = pl[i].y - pl[i + 1].y;// 得到两点构成
        vz = pl[i].z - pl[i + 1].z;// 的向量
        types[i].dista = vx * vx + vy * vy + vz * vz;// 向量模长(少开根号)
#ifdef DEBUG
        std::cout<<vx<<" "<<vx<<" "<<vz<<" "<<std::endl;
#endif
    }
    // plsize++;
    // 设置最后一个点的range信息 : = 最后一点到原点的距离
    types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);

    // 提取角点特征(pl_corn)和面特征(pl_surf)
    give_feature(pl, types, pl_corn, pl_surf);

    // 将转换后的pl,提取到的pl_surf,pl_corn发布出去
    ros::Time ct(ros::Time::now());
    pub_func(pl, pub_full, msg->header.stamp);
    pub_func(pl_surf, pub_surf, msg->header.stamp);
    pub_func(pl_corn, pub_corn, msg->header.stamp);
}

void horizon_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    double t1 = omp_get_wtime();
    vector <pcl::PointCloud<PointType>> pl_buff(N_SCANS); // 按线分的点云，去掉了重复的点
    vector <vector<orgtype>> typess(N_SCANS);
    // 全部的原始点云，use curvature as time of each laser points；
    pcl::PointCloud <PointType> pl_full, pl_corn, pl_surf;// pl_surf：降采样的原始点云

    uint plsize = msg->point_num; // all point's number

    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    for (int i = 0; i < N_SCANS; i++) {
        pl_buff[i].reserve(plsize);
    }
    // ANCHOR - remove nearing pts.
    for (uint i = 1; i < plsize; i++) {
        // clang-format off
        if ((msg->points[i].line < N_SCANS)
            && (!IS_VALID(msg->points[i].x)) // 无效点
            && (!IS_VALID(msg->points[i].y))
            && (!IS_VALID(msg->points[i].z))
            && msg->points[i].x > 0.7) { // 盲区点
            // https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol
            // See [3.4 Tag Information] 
            if ((msg->points[i].x > 2.0)
                && (((msg->points[i].tag & 0x03) != 0x00) || ((msg->points[i].tag & 0x0C) != 0x00))
                    ) {
                // Remove the bad quality points
                continue;
            }
            // clang-format on
            pl_full[i].x = msg->points[i].x;
            pl_full[i].y = msg->points[i].y;
            pl_full[i].z = msg->points[i].z;
            pl_full[i].intensity = msg->points[i].reflectivity;

            pl_full[i].curvature =
                    msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points

            if ((std::abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
                (std::abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
                (std::abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) {
                pl_buff[msg->points[i].line].push_back(pl_full[i]);
            }
        }
    }
    if (pl_buff.size() != N_SCANS) {
        return;
    }
    if (pl_buff[0].size() <= 7) {
        return;
    }
    for (int j = 0; j < N_SCANS; j++) {
        pcl::PointCloud <PointType> &pl = pl_buff[j];
        vector <orgtype> &types = typess[j];
        plsize = pl.size();
        if (plsize < 7) {
            continue;
        }
        types.resize(plsize);
        plsize--;
        for (uint pt_idx = 0; pt_idx < plsize; pt_idx++) {
            types[pt_idx].range = pl[pt_idx].x * pl[pt_idx].x + pl[pt_idx].y * pl[pt_idx].y;
            vx = pl[pt_idx].x - pl[pt_idx + 1].x;
            vy = pl[pt_idx].y - pl[pt_idx + 1].y;
            vz = pl[pt_idx].z - pl[pt_idx + 1].z;

            // std::cout<<vx<<" "<<vx<<" "<<vz<<" "<<std::endl;
        }
        // plsize++;
        types[plsize].range = pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y;
        give_feature(pl, types, pl_corn, pl_surf);
    }
    if (pl_surf.points.size() < 100) {
        return;
    }
    ros::Time ct;
    ct.fromNSec(msg->timebase);
    pub_func(pl_full, pub_full, msg->header.stamp);
    pub_func(pl_surf, pub_surf, msg->header.stamp);
    pub_func(pl_corn, pub_corn, msg->header.stamp);
}

int orders[16] = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15};

class Point;
namespace velodyne_ros{
    //EIGEN_ALIGN16定义在 Eigen/src/Core/util/Macros.h 中的一个宏用来做数据对齐
    struct EIGEN_ALIGN16 Point{
        PCL_ADD_POINT4D;                // 4D点坐标类型(POINT_CLOUD_REGISTER_POINT_STRUCT的必须)
        float intensity;                // 强度
        float time;                     // 时间
        uint16_t ring;                  // 点所属的圈数
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
    };
}
// 注册velodyne_ros的Point类型(pcl新增自定义点云类型)
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (float, time, time)
                                          (std::uint16_t, ring, ring)
)
// clang-format on
void velo16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // TODO
    float time_unit_scale;
    pcl::PointCloud<PointType> pl_surf, pl_corn, pl_full;
    pcl::PointCloud<PointType> pl_buff[128];//maximum 128 line lidar
    vector<orgtype>typess[128];//maximum 128 line lidar
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    //10Hz 1s转360度 1ms转0.36度
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/
    //todo check pl_orig.points[plsize - 1].time
    if(pl_orig.points[plsize-1].time > 0){//假如提供了每个点的时间戳
        given_offset_time = true;
    }else{//没有提供每个点的时间戳
        given_offset_time = false;
        //atan2:返回给定的 X 及 Y 坐标值的反正切值。
        // atan2(a, b) 与 atan(a/b)稍有不同，atan2(a,b)的取值范围介于 -pi 到 pi 之间（不包括 -pi）
        //而atan(a/b)的取值范围介于-pi/2到pi/2之间（不包括±pi/2)。
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;// 记录第一个点(index 0)的yaw， to degree
        double yaw_end = yaw_first;//TODO??
        int layer_first = pl_orig.points[0].ring;// 第一个点(index 0)的layer序号
        for(uint i = plsize - 1;i > 0;i--){ // 倒序遍历，找到与第一个点相同layer的最后一个点
            if(pl_orig.points[i].ring == layer_first){
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;//寻找终止yaw角
                break;
            }
        }
    }

    if(feature_enabled){
        for(int i = 0;i < N_SCANS;i++){
            pl_buff[i].clear();
            pl_buff[i].reserve(plsize);
        }
        //计算时间、转换点云格式为PointType，正序遍历
        for(int i = 0;i < plsize;i++){
            PointType added_pt;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            int layer = pl_orig.points[i].ring;
            if (layer >= N_SCANS) continue;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
//            added_pt.curvature = pl_orig.points[i].time/1000.0; // units: ms
            added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // units: ms

            if(!given_offset_time){//假如没有提供时间
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
                if(is_first[layer]){//给每条线第一个赋予yaw角 时间归0
#ifdef DEBUG
                    printf("layer: %d; is first: %d", layer, is_first[layer]);
#endif
                    yaw_fp[layer] = yaw_angle;// 记录为当前点对应layer的起始yaw
                    is_first[layer] = false;
                    added_pt.curvature = 0.0; //当前点curvature（时间）置零
                    yaw_last[layer] = yaw_angle;//对每一条线最后一个点的时间赋值( 暂时记录为当前点对应layer的结束yaw)
                    time_last[layer] = added_pt.curvature;//对每一条线最后一个点的时间赋值
                    continue;
                }
                //对每个点计算时间
                if(yaw_angle <= yaw_fp[layer]){
                    added_pt.curvature = (yaw_fp[layer] - yaw_angle)/omega_l;
                }else{
                    added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0)/omega_l;
                }
                //新得到的点的时间不能早于之前的最后一个点的时间
                if(added_pt.curvature < time_last[layer]) added_pt.curvature+=360.0/omega_l;

                yaw_last[layer] = yaw_angle;// 记录当前layer最后一个点的yaw
                time_last[layer] = added_pt.curvature;//  记录当前layer最后一个点的时间
            }
            pl_buff[layer].points.push_back(added_pt);
        }
        for(int j = 0;j < N_SCANS;j++){
            pcl::PointCloud<PointType> &pl = pl_buff[j];
            int linesize = pl.size();
            if(linesize < 2) continue;//点太少跳过
//            vector<orgtype> &types = typess[j];
            vector <orgtype> &types = typess[j];
            types.clear();
            types.resize(linesize);
            linesize--;
            for(uint i = 0;i < linesize;i++){
                types[i].range = sqrt(pl[i].x * pl[i].x +pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i+1].x;
                vy = pl[i].y - pl[i+1].y;
                vz = pl[i].z - pl[i+1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
            give_feature(pl, types, pl_corn, pl_surf);
        }
    }else{
        for(int i = 0;i < plsize;i++){
            PointType added_pt;
#ifdef DEBUG
            cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
#endif
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
//            added_pt.curvature = pl_orig.points[i].time / 1000.0;  // curvature unit: ms
            added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms
#ifdef DEBUG
            cout<<added_pt.curvature<<endl;
#endif
            if(!given_offset_time){
                int layer = pl_orig.points[i].ring;
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
                if(is_first[layer]){
#ifdef DEBUG
                    printf("layer: %d; is first: %d", layer, is_first[layer]);
#endif
                    yaw_fp[layer]=yaw_angle;
                    is_first[layer]=false;
                    added_pt.curvature = 0.0;
                    yaw_last[layer]=yaw_angle;
                    time_last[layer]=added_pt.curvature;
                    continue;
                }
                // compute offset time
                if (yaw_angle <= yaw_fp[layer])
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
                }
                else
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
                }

                if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

                yaw_last[layer] = yaw_angle;
                time_last[layer]=added_pt.curvature;
            }
            // 等间隔降采样
            if(i % point_filter_num == 0){
                if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind)){
                    pl_surf.points.push_back(added_pt);
                }
            }
        }
    }
//    ros::Time ct;
//    ct.fromNSec(msg->timebase);
//    pub_func(pl_full, pub_full, msg->header.stamp);
//    pub_func(pl_surf, pub_surf, msg->header.stamp);
//    pub_func(pl_corn, pub_corn, msg->header.stamp);
    ros::Time ct(ros::Time::now());
    pub_func(pl_full, pub_full, msg->header.stamp);
    pub_func(pl_surf, pub_surf, msg->header.stamp);
    pub_func(pl_corn, pub_corn, msg->header.stamp);

}

void velo16_handler1(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // TODO
}

namespace ouster_ros {

    struct EIGEN_ALIGN16 Point{
            PCL_ADD_POINT4D;
            float intensity;
            uint32_t t;
            uint16_t reflectivity;
            uint8_t ring;
            uint16_t ambient;
            uint32_t range;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
// use std::uint32_t to avoid conflicting with pcl::uint32_t
(std::uint32_t, t, t)
(std::uint16_t, reflectivity, reflectivity)
(std::uint8_t, ring, ring)
(std::uint16_t, ambient, ambient)
(std::uint32_t, range, range)
)
// clang-format on

void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud <PointType> pl_processed;
    pcl::PointCloud <ouster_ros::Point> pl_orig;
    // pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    uint plsize = pl_orig.size();

    double time_stamp = msg->header.stamp.toSec();
    pl_processed.clear();
    pl_processed.reserve(pl_orig.points.size());
    for (int i = 0; i < pl_orig.points.size(); i++) {
        double range = std::sqrt(pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                                 pl_orig.points[i].z * pl_orig.points[i].z);
        if (range < blind) {
            continue;
        }
        Eigen::Vector3d pt_vec;
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.3;
        if (yaw_angle >= 180.0)
            yaw_angle -= 360.0;
        if (yaw_angle <= -180.0)
            yaw_angle += 360.0;

        added_pt.curvature = (pl_orig.points[i].t / 1e9) * 1000.0;

        pl_processed.points.push_back(added_pt);
        if (0) // For debug
        {
            if (pl_processed.size() % 1000 == 0) {
                printf("[%d] (%.2f, %.2f, %.2f), ( %.2f, %.2f, %.2f ) | %.2f | %.3f,  \r\n", i, pl_orig.points[i].x,
                       pl_orig.points[i].y,
                       pl_orig.points[i].z, pl_processed.points.back().normal_x, pl_processed.points.back().normal_y,
                       pl_processed.points.back().normal_z, yaw_angle, pl_processed.points.back().intensity);
                // printf("(%d, %.2f, %.2f)\r\n", pl_orig.points[i].ring, pl_orig.points[i].t, pl_orig.points[i].range);
                // printf("(%d, %d, %d)\r\n", pl_orig.points[i].ring, 1, 2);
                cout << (int) (pl_orig.points[i].ring) << ", " << (pl_orig.points[i].t / 1e9) << ", "
                     << pl_orig.points[i].range << endl;
            }
        }
    }
    pub_func(pl_processed, pub_full, msg->header.stamp);
    pub_func(pl_processed, pub_surf, msg->header.stamp);
    pub_func(pl_processed, pub_corn, msg->header.stamp);
}

/**提取角点特征和面点特征
 * pl : Lidar帧的原始点容器
 * types : 对Lidar原始点计算了相邻两点距离后的点容器
 * pl_corn : 存放提取到的点特征
 * pl_surf : 存放提取到的面特征
 * g_if_using_raw_point == 1 使用原始点云 pl_surf = pl, pl_corn = 空
 * ***注意*** : 默认没有提取点特征,且只以间隔3个点的形式赋值了面特征
 *             可能是为了速度吧~~~
 */
void give_feature(pcl::PointCloud <PointType> &pl, vector <orgtype> &types, pcl::PointCloud <PointType> &pl_corn,
                  pcl::PointCloud <PointType> &pl_surf) {
    /**
     * @note (1)预先判断点数量和点的距离,并定义相关变量
     */
    uint plsize = pl.size();
    uint plsize2;
    if (plsize == 0) {
        printf("something wrong\n");
        return;
    }
    uint head = 0;
    // 跳过盲区点云
    while (types[head].range < blind) {// blind = 0.1 : default
        // types[i].range = pl[i].x
        head++; // 因此猜测这里判断lidar帧中点的距离
        // 保证后续处理的起点在一定距离外,防止干扰点
    }

    // Surf
    // Surf : group_size = 8 : 点数大于8,则后续处理范围为 head -> 点数-8,
    // 否则为0, 应该是只计算head到倒数第8点之间的中间点
    plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

    uint i_nex = 0, i2;
    uint last_i = 0;
    uint last_i_nex = 0;
    int last_state = 0;
    int plane_type;

    // 充当中间交换量的临时容器
    PointType ap;
    /**
     * @note (2)
     *  对head到倒数第8点之间的中间点进行计算
     */
    // g_LiDAR_sampling_point_step = 3
    //这里以间隔3个点的形式赋值了面特征，证明了后面并没有进行特征提取
    for (uint i = head; i < plsize2; i += g_LiDAR_sampling_point_step){
        if (types[i].range > blind){
            ap.x = pl[i].x;
            ap.y = pl[i].y;
            ap.z = pl[i].z;
            ap.curvature = pl[i].curvature;
            pl_surf.push_back(ap); // pl_surf : 存放提取到的面特征
        }
        if (g_if_using_raw_point){
            continue;
        }
        // i_nex = i;
        i2 = i;
#ifdef DEBUG
//        i_nex = i;
        std::cout << " i: " << i << " i_nex: " << i_nex << "group_size: " << group_size << " plsize: " << plsize
                  << " plsize2: " << plsize2 << std::endl;
#endif
        plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

        if (plane_type == 1){
            for (uint j = i; j <= i_nex ; j++) {
                if ( j != i && j != i_nex ){ // 不在群的首尾
                    types[ j ].ftype = Real_Plane;
                }else{
                    types[ j ].ftype = Poss_Plane;
                }
            }
            // if(last_state==1 && fabs(last_direct.sum())>0.5)
            //last_state == 1上一个群是平面 && last_direct.norm() > 0.1 表示有上一次的方向
            if ( last_state == 1 && last_direct.norm() > 0.1 )
            {
                //TODO:???
                double mod = last_direct.transpose() * curr_direct;// 夹角
                if ( mod > -0.707 && mod < 0.707 )// 夹角大于45°
                {
                    types[ i ].ftype = Edge_Plane;
                }
                else
                {
                    types[ i ].ftype = Real_Plane;
                }
            }

            i = i_nex - 1;
            last_state = 1;
        }else if (plane_type == 2){ //blind点
            i = i_nex;
        }else if (plane_type == 0){
            if ( last_state == 1 )
            {
                uint i_nex_tem;
                uint j;
                for ( j = last_i + 1; j <= last_i_nex; j++ )
                {
                    uint            i_nex_tem2 = i_nex_tem;
                    Eigen::Vector3d curr_direct2;

                    uint ttem = plane_judge( pl, types, j, i_nex_tem, curr_direct2 );

                    if ( ttem != 1 )
                    {
                        i_nex_tem = i_nex_tem2;
                        break;
                    }
                    curr_direct = curr_direct2;
                }

                if ( j == last_i + 1 )
                {
                    last_state = 0;
                }
                else
                {
                    for ( uint k = last_i_nex; k <= i_nex_tem; k++ )
                    {
                        if ( k != i_nex_tem )
                        {
                            types[ k ].ftype = Real_Plane;
                        }
                        else
                        {
                            types[ k ].ftype = Poss_Plane;
                        }
                    }
                    i = i_nex_tem - 1;
                    i_nex = i_nex_tem;
                    i2 = j - 1;
                    last_state = 1;
                }
            }
        }

        last_i = i2;
        last_i_nex = i_nex;
        last_direct = curr_direct;
    }
    if (g_if_using_raw_point) {//g_if_using_raw_point=1
        return;
    }
    plsize2 = plsize > 3 ? plsize - 3 : 0;
    for (uint i = head + 3; i < plsize2; i++) {
        if (types[i].range < blind || types[i].ftype >= Real_Plane) {
            continue;
        }

        if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) {
            continue;
        }

        Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
        Eigen::Vector3d vecs[2];

        for (int j = 0; j < 2; j++) {
            int m = -1;
            if (j == 1) {
                m = 1;
            }

            if (types[i + m].range < blind) {
                if (types[i].range > inf_bound) {
                    types[i].edj[j] = Nr_inf;
                } else {
                    types[i].edj[j] = Nr_blind;
                }
                continue;
            }

            vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
            vecs[j] = vecs[j] - vec_a;

            types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
            if (types[i].angle[j] < jump_up_limit) {
                types[i].edj[j] = Nr_180;
            } else if (types[i].angle[j] > jump_down_limit) {
                types[i].edj[j] = Nr_zero;
            }
        }

        types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
        if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 &&
            types[i].dista > 4 * types[i - 1].dista) {
            if (types[i].intersect > cos160) {
                if (edge_jump_judge(pl, types, i, Prev)) {
                    types[i].ftype = Edge_Jump;
                }
            }
        } else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 &&
                   types[i - 1].dista > 4 * types[i].dista) {
            if (types[i].intersect > cos160) {
                if (edge_jump_judge(pl, types, i, Next)) {
                    types[i].ftype = Edge_Jump;
                }
            }
        } else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf) {
            if (edge_jump_judge(pl, types, i, Prev)) {
                types[i].ftype = Edge_Jump;
            }
        } else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor) {
            if (edge_jump_judge(pl, types, i, Next)) {
                types[i].ftype = Edge_Jump;
            }
        } else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor) {
            if (types[i].ftype == Nor) {
                types[i].ftype = Wire;
            }
        }
    }

    plsize2 = plsize - 1;
    double ratio;
    for (uint i = head + 1; i < plsize2; i++) {
        if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind) {
            continue;
        }

        if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) {
            continue;
        }

        if (types[i].ftype == Nor) {
            if (types[i - 1].dista > types[i].dista) {
                ratio = types[i - 1].dista / types[i].dista;
            } else {
                ratio = types[i].dista / types[i - 1].dista;
            }

            if (types[i].intersect < smallp_intersect && ratio < smallp_ratio) {
                if (types[i - 1].ftype == Nor) {
                    types[i - 1].ftype = Real_Plane;
                }
                if (types[i + 1].ftype == Nor) {
                    types[i + 1].ftype = Real_Plane;
                }
                types[i].ftype = Real_Plane;
            }
        }
    }

    int last_surface = -1;
    for (uint j = head; j < plsize; j++) {
        if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane) {
            if (last_surface == -1) {
                last_surface = j;
            }

            if (j == uint(last_surface + point_filter_num - 1)) {
                PointType ap;
                ap.x = pl[j].x;
                ap.y = pl[j].y;
                ap.z = pl[j].z;
                ap.curvature = pl[j].curvature;
                pl_surf.push_back(ap);

                last_surface = -1;
            }
        } else {
            if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) {
                pl_corn.push_back(pl[j]);
            }
            if (last_surface != -1) {
                PointType ap;
                for (uint k = last_surface; k < j; k++) {
                    ap.x += pl[k].x;
                    ap.y += pl[k].y;
                    ap.z += pl[k].z;
                    ap.curvature += pl[k].curvature;
                }
                ap.x /= (j - last_surface);
                ap.y /= (j - last_surface);
                ap.z /= (j - last_surface);
                ap.curvature /= (j - last_surface);
                pl_surf.push_back(ap);
            }
            last_surface = -1;
        }
    }
}

void pub_func(pcl::PointCloud <PointType> &pl, ros::Publisher pub, const ros::Time &ct) {
    pl.height = 1;
    pl.width = pl.size();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pl, output);// 转换为PointCloud2再发布
    output.header.frame_id = "livox";
    output.header.stamp = ct;
    pub.publish(output);
}

/**
 * @brief    判断一个点是否是平面点，计算群的方向
 *
 * @param pl            全部点云
 * @param types
 * @param i_cur         当前点索引
 * @param i_nex         同一个群里的下一个点索引（退出时为下一个群的起点）
 * @param curr_direct   群内第一个点指向最后一个点的方向向量
 * @return int   0不是 1平面  2盲区
 */
int plane_judge(const pcl::PointCloud <PointType> &pl, vector <orgtype> &types, uint i_cur, uint &i_nex,
                Eigen::Vector3d &curr_direct) {
    // 计算一个群的距离
    double group_dis = disA * types[i_cur].range + disB;
    group_dis = group_dis * group_dis;
    // i_nex = i_cur;

    double two_dis;
    vector<double> disarr; // 当前点和下一个点之间的距离
    disarr.reserve(20);

    for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++) {
        if (types[i_nex].range < blind) {
            curr_direct.setZero();
            return 2;
        }
        disarr.push_back(types[i_nex].dista);
    }

    // 无线循环直到距离内的点都加入进一个群
    for (;;) {
        if ((i_cur >= pl.size()) || (i_nex >= pl.size()))
            break;

        if (types[i_nex].range < blind) {
            curr_direct.setZero();
            return 2;
        }
        vx = pl[i_nex].x - pl[i_cur].x;
        vy = pl[i_nex].y - pl[i_cur].y;
        vz = pl[i_nex].z - pl[i_cur].z;
        two_dis = vx * vx + vy * vy + vz * vz;
        if (two_dis >= group_dis) {
            break;
        }
        disarr.push_back(types[i_nex].dista);
        i_nex++;
    }

    // 退出上面循环后，vx,vy,vz代表群内第一个点指向最后一个点的方向
    double leng_wid = 0; // 存储最大的垂直距离
    double v1[3], v2[3];
    // 遍历群中每一个点
    for (uint j = i_cur + 1; j < i_nex; j++) {
        if ((j >= pl.size()) || (i_cur >= pl.size()))
            break;
        v1[0] = pl[j].x - pl[i_cur].x;
        v1[1] = pl[j].y - pl[i_cur].y;
        v1[2] = pl[j].z - pl[i_cur].z;

        //叉乘
        v2[0] = v1[1] * vz - vy * v1[2];
        v2[1] = v1[2] * vx - v1[0] * vz;
        v2[2] = v1[0] * vy - vx * v1[1];

        double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
        if (lw > leng_wid) {
            leng_wid = lw;
        }
    }

    if ((two_dis * two_dis / leng_wid) < p2l_ratio) {
        curr_direct.setZero();
        return 0;
    }

    // 按照从大到小排序
    uint disarrsize = disarr.size();
    for (uint j = 0; j < disarrsize - 1; j++) {
        for (uint k = j + 1; k < disarrsize; k++) {
            if (disarr[j] < disarr[k]) {
                leng_wid = disarr[j];
                disarr[j] = disarr[k];
                disarr[k] = leng_wid;
            }
        }
    }

    if (disarr[disarr.size() - 2] < 1e-16) {
        curr_direct.setZero();
        return 0;
    }

    if (lidar_type == MID || lidar_type == HORIZON) {
        double dismax_mid = disarr[0] / disarr[disarrsize / 2]; // 最大值/中值
        double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2]; // 中值/最小值

        if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin) {
            curr_direct.setZero();
            return 0;
        }
    } else {
        double dismax_min = disarr[0] / disarr[disarrsize - 2];
        if (dismax_min >= limit_maxmin) {
            curr_direct.setZero();
            return 0;
        }
    }

    curr_direct << vx, vy, vz;
    curr_direct.normalize();
    return 1;
}

bool edge_jump_judge(const pcl::PointCloud <PointType> &pl, vector <orgtype> &types, uint i, Surround nor_dir) {
    if (nor_dir == 0) {
        if (types[i - 1].range < blind || types[i - 2].range < blind) {
            return false;
        }
    } else if (nor_dir == 1) {
        if (types[i + 1].range < blind || types[i + 2].range < blind) {
            return false;
        }
    }
    double d1 = types[i + nor_dir - 1].dista;
    double d2 = types[i + 3 * nor_dir - 2].dista;
    double d;

    if (d1 < d2) {
        d = d1;
        d1 = d2;
        d2 = d;
    }

    d1 = sqrt(d1);
    d2 = sqrt(d2);

    if (d1 > edgea * d2 || (d1 - d2) > edgeb) {
        return false;
    }

    return true;
}
