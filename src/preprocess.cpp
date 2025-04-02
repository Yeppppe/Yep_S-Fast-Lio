#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS = 6;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  disA = 0.1; // B?
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  //* laserMapping通过参数初始化time_unit为US  即单位尺度因子为us 正常时间戳为微妙 乘以尺度因子后转换为ms
  //* 为什么时间统计单位为us?
  switch (time_unit)
  {
  case SEC:
    time_unit_scale = 1.e3f;
    break;
  case MS:
    time_unit_scale = 1.f;
    break;
  //*  1.e-3f = 0.001 千分之一
  case US:
    time_unit_scale = 1.e-3f;
    break;
  case NS:
    time_unit_scale = 1.e-6f;
    break;
  default:
    time_unit_scale = 1.f;
    break;
  }

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;
  //* 估计后面会用16线的雷达
  case VELO16:
    velodyne_handler(msg);
    break;

  case RS32:
    rs_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  //* 只输出平面点
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;
  // cout<<"plsie: "<<plsize<<endl;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  if (feature_enabled)
  {
    for (uint i = 1; i < plsize; i++)
    {
      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points

        bool is_new = false;
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count++;
    double t0 = omp_get_wtime();
    for (int j = 0; j < N_SCANS; j++)
    {
      if (pl_buff[j].size() <= 5)
        continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for (uint i = 0; i < plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types);
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else
  {
    for (uint i = 1; i < plsize; i++)
    {
      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num++;
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
          // std::cout << "pl_full[i].curvature: " << pl_full[i].curvature << std::endl;

          if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind))
        continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;
      if (pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0)
        continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

      if (range < (blind * blind))
        continue;

      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

//* velodyne16点云预处理函数
//* velodune点云格式？ ----> 飞书问题Todo
void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  //* pl_xxx的数据类型是点云类型PointCloudXYZI
  //* 清空现有的点云数据
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  //* 点云数据转换  ros:msg  ----> pcl:pl_orig数据
  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;
  pl_surf.reserve(plsize);

  /*** These variables only works when no point timestamps given ***/
  double omega_l = 0.361 * SCAN_RATE; //* scan angular velocity  SCAN_RATE = 10 单位是度/ms 相当于每秒钟转3600度
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);   // yaw of first scan point
  std::vector<float> yaw_last(N_SCANS, 0.0);  // yaw of last scan point
  std::vector<float> time_last(N_SCANS, 0.0); // last offset time
  /*****************************************************************/

  //* 时间戳处理判断
  //* 检查点云是否包含时间戳信息
  if (pl_orig.points[plsize - 1].time > 0)   //* >0说明点云自带时间戳信息
  {
    given_offset_time = true;
  }
  //*  计算一圈扫描点的初始角度和结束角度 同时设置 given_offset_time = false
  else
  {
    given_offset_time = false;
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;   //* 计算第一个点的偏航角，* 57.29578表示将弧度转换为角度
    double yaw_end = yaw_first; 
    //* 获取第一个点所在的扫描线    这个ring是一开始就有的吗？
    int layer_first = pl_orig.points[0].ring;
    //* 这样找是不是最慢的找法？
    for (uint i = plsize - 1; i > 0; i--)
    {
      //* 从最后一个点往前查找，知道找到第一条扫描线的最后一个点
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  //* 特征提取模式   fast_lio2默认不进行特征点提取
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      //* process.h中的定义PointCloudXYZI pl_buff[128]
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    //* 这个for循环已经遍历一遍点，添加了时间戳以及x , y , z信息到pl_buff[layer]中
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      //* pl_orig就是原始的点云数据
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS)
        continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale; //* 现在为止所有的点的时间可能都是ros时间戳赋予的时间，即一帧所有点的时间戳均相同 并转换成秒，不过无所谓，下面你就立刻变为0了

      //* 开始计算每个点的时间戳
      if (!given_offset_time)
      {
        //* 注意atan2于atan不同，它能够计算的角度范围为(-180,180] ,且能够根据x和y的符号判断象限
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        //* 将每层第一个点的时间戳记为0 
        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          //* 每层的第一个点会把当层后续点的标志置为false
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }
        if (yaw_angle <= yaw_fp[layer])
        {
          //* omega_l是雷达旋转的角速度 单位是度/ms意思是 转换过来就是1秒钟转3610度，也就是每秒10圈
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        //* 当前角度的时间戳小于上一次的时间，可能是跨越了象限 把角度从[-180-180]，需要补上360度的增量的时间
        //? 所以雷达扫描的时候 是一圈一圈扫16圈算一帧 还是一圈就能扫完16线 成为一帧？
        //* 答 16线表示包含16个激光扫描器，共同形成一帧  这个应该是怕角度为负数，对应扫描时间也是负的了

        //! 这行代码执行的时候，当前扫描时间小于上一次扫描时间，那是进入新的一圈的时候吗，那时间直接增加0.1s？
        //* 答：肯定不是，到下一次的时候layer都变了，每一层的第一个点永远是0，这个可能是为了保险起见？ 或者某一层的最后一个雷达扫描点超过了起始点？
        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        //* 用来跟踪没曾扫描点最近处理的点的偏航角和时间戳
        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    //* 遍历每条扫描线 pl_buff[j]存储了第j条扫描线上的所有的点
    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      //* linesize 是一条扫描线的点云个数
      int linesize = pl.size();
      if (linesize < 2)
        continue;
      //* orgtype是存储点特征的结构体
      //* typess数据类型vector<orgtype> typess[128]
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      //* 填补types特征 range和dista 这个range是水平面上投影距离，不包括z轴
      for (uint i = 0; i < linesize; i++)
      {
        //* 计算扫描点的距离 以及相邻点的距离
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      //* 单独计算最后一个点的 距原点的距离 因为最后一个点没有下一个点 因此不会计算dista
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);

      //* give_feature()的输入是一条线的点云以及该线点云的特征 其中特征只包含range和dista
      give_feature(pl, types);
    }
  }
  else
  {
    for (int i = 0; i < plsize; i++)
    {
      //* typedef pcl::PointXYZINormal PointType;
      //* pcl::PointXYZINormal数据类型  XYZ:位置信息  I:intensity强度信息  Normal:法向量  curvature:曲率
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      //! 这里的intensity是什么信息？
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // curvature unit: ms // cout<<added_pt.curvature<<endl;
      // std::cout << "added_pt.curvature:" << added_pt.curvature << std::endl;


      if (!given_offset_time)  //* 给一帧点赋予时间戳
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }
      //* point_filter_num初始化为1 那岂不是不滤除点
      if (i % point_filter_num == 0)
      {
        //* 提出盲点
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}

//* 从雷达中提取平面点和边缘点
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  //* plsize是一根线的点的数量
  int plsize = pl.size();
  int plsize2;
  if (plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;
  //* 查找第一个非盲点
  while (types[head].range < blind)
  {
    head++;
  }

  // Surf
  //* group_size = 8
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0;
  uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  //* 判断Real_Plane，Poss_Plane，Edge_Plane点  平面段中间的点一定为Real_Plane 平面段两头的可能为Edge_Plane或Poss_Plane
  for (uint i = head; i < plsize2; i++)
  {
    //* 过滤掉盲点
    if (types[i].range < blind)
    {
      continue;
    }

    i2 = i;
    //* 判断从i->i_nex是否可能构成一个平面  plane_type = 2不为平面点
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    //* 当前点为平面点 盘点i--->i_nex的点的类型 ：Real_Plane  Poss_Plane   Edge_Plane
    if (plane_type == 1)
    {
      //* plane_judge如果判断为平面点，则从i->i_nex为细长平面
      for (uint j = i; j <= i_nex; j++)
      {
        //* 标记为平面点 的中间部分点
        if (j != i && j != i_nex)
        {
          types[j].ftype = Real_Plane;  //* 实平面
        }
        else
        {
          types[j].ftype = Poss_Plane;   //* 细长平面点的首尾平面点，标记为可能平面点
        } 
      }

      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      //* 检查与上一个平面的关系  last_state == 1表示上一段点云判定为平面   last_direct.norm()>0.1表示方向向量长度足够可靠
      //* 如果上一段判定为平面 ，则其方向向量一定是已经归一化后的
      //* 通过与上一段平面点的关系判断为边缘平面还是普通实平面点
      if (last_state == 1 && last_direct.norm() > 0.1)
      {
        //* 当前方向向量与上一个方向向量的点积 因为是单位向量 即mod = cos(theta)
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707)    //* 夹角在45 到 135之间
        {
          types[i].ftype = Edge_Plane;      //* 判定为两平面的角线点，也是角点
        }
        else
        {
          types[i].ftype = Real_Plane;       //* 判断为普通平面
        }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    //* if(plane_type == 2) 下次遍历从i_nex开始
    else 
    {
      i = i_nex;          //* 对于非平面直接跳到扩展的终点
      last_state = 0;     //* 标记为非平面状态
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    //* last_i是上一次的的开始点   last_i_nex:如果是平面点则表示为平面搜索结束点  last_direct上一次若为平面点 则为主方向向量，否则就为0
    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  //* 截断掉最前面三个和最后面三个点
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < plsize2; i++)
  {
    //* 跳过盲区点 和 已经判定为平面点的点
    // enum Feature
    // {
    //   Nor,
    //   Poss_Plane,
    //   Real_Plane,
    //   Edge_Jump,
    //   Edge_Plane,
    //   Wire,
    //   ZeroPoint
    // };
    //* 只处理 Nor Poss_Plane Real_Plane
    if (types[i].range < blind || types[i].ftype >= Real_Plane)
    {
      continue;
    }

    //* 跳过距离过近的点 （过于密集）
    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    //* 同一个代码处理前后两个相邻点，减少代码重复
    for (int j = 0; j < 2; j++)   //* j=0 m=-1    j=1 m=1
    {
      int m = -1;
      if (j == 1)
      {
        m = 1;
      }
      //* blind通过类初始化为0.01  inf_bound定义为10
      //* 第一遍for循环检查的是前一个点为盲点，当前点大于inf_bound 则标记当前点的edj[pre]=Nr_inf无穷点  
      if (types[i + m].range < blind)
      {
        if (types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;   //* 相邻点为盲点，当前点大于10m则直接标记为无穷点
        }
        else
        {
          types[i].edj[j] = Nr_blind;     //* 相邻点为盲点，当前点距离不超过10m则也标记为盲点，多少带点连带责任了
        }
        continue;
      }

      //* 计算从当前点到相邻点的向量
      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a;   //* vec_a表示的是当前点 那 vec[j]则代表当前点到相邻点的向量

      //* 计算两个向量夹角的余弦值 原点到当前点以及当前点到相邻点的向量
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if (types[i].angle[j] < jump_up_limit)   //* jump_up_limit是cos(170/180*pi)  其值在0-pi是单调递减的 即夹角大于170°
      {
        types[i].edj[j] = Nr_180;    //* 接近180的边缘
      }
      else if (types[i].angle[j] > jump_down_limit)    //* jump_down_limit = cos(8/180*pi) 其值在0-pi是单调递减的 即其夹角小于8°
      {  
        types[i].edj[j] = Nr_zero;   //* 接近0°的边缘
      }
    }

    //* 计算当前点到前一点所构成的向量与当前点到后一点所构成向量的夹角余弦值
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    //* 第一种跳变情况 ：前一点正常，后一点接近0度 当前点与后一点间距大于0.0225 并且 是前一点间距的四倍以上
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      //* 两向量夹角大于160°
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;    //* 标记为边缘跳变点
        }
      }
    }
    //* 第二种情况，前一点零度 后一点正常
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    //* 前一点正常，后一点无穷
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    //* 前一点无穷，后一点正常
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    //* 前后都不是正常点 但当前点是普通点   
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor)
      {
        types[i].ftype = Wire;  //* 标记为线性特征
      }
    }
  }

  plsize2 = plsize - 1;
  double ratio;
  
  for (uint i = head + 1; i < plsize2; i++)   //* 剩余Nor 不是特别密集的都变为平面点了
  {
    //* 跳过盲区点
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind)
    {
      continue;
    }
    //* 跳过距离过小的点
    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
    {
      continue;
    }
    //* 处理还没有被特殊处理过的点   .ftype默认几位Nor
    if (types[i].ftype == Nor)
    {
      //* 计算距离比，始终是大除以小
      if (types[i - 1].dista > types[i].dista)
      {
        ratio = types[i - 1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i - 1].dista;
      }

      //* types[i].intersect是什么?
      //* 判断是否为平面点
      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        if (types[i - 1].ftype == Nor)
        {
          types[i - 1].ftype = Real_Plane;
        }
        if (types[i + 1].ftype == Nor)
        {
          types[i + 1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  //* 特征提取和降采样
  int last_surface = -1;
  for (uint j = head; j < plsize; j++)
  {
    //* 对于可能是平面点 进行降采样处理
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      if (last_surface == -1)
      {
        last_surface = j;
      }
      //? 这部分为什么能降采样不太清楚
      //* 比如说point_filter = 2 那第一次来到平面点的时候不满足if条件 但是last_surface->j 下次来到j+1点的时候，last_surface还是j 就满足此if条件了！ 然后last_surface正常清0 
      if (j == uint(last_surface + point_filter_num - 1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    //* 处理非平面点 保存边缘点----> pl_corn  同时压缩一系列非平面点非角点的点 进入pl_surf
    else
    {
      //* 保存边缘点----> pl_corn
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      //* 计算从last_surface到当前点j之前的所有平面点的平均值 这部分没太懂 ，前面大部分平面点应该都被降采样过了，难道是防止降采样过大？
      if (last_surface != -1)  //* 处理到非平面点 且之前有连续的平面点
      {
        PointType ap;
        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

//* 判断是否可以构成一个平面
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  //* disB没给赋值，默认为0?
  //* disA = 0.1
  //* group_dis会随着点到激光雷达的距离线性增加
  double group_dis = disA * types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  //* group_size 初始化为8
  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }

  //* 持续扩展inex，直到i_cur与i_nex之间的距离超过了group_dis阈值
  for (;;)
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size()))
      break;

    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    //* 因为上面已经默认添加了8个点，因此这个持续循环的点是从第九个点开始获取的
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;
    //*两点之间超过了阈值 就会退出循环
    //* 有点类似水平方向的bfs
    if (two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }
  //* 到上述代码结束 已经选择了一个初步的区间 从i_cur---->i_nex可能会构成一个平面

  double leng_wid = 0;
  double v1[3], v2[3];
  //! 下述代码是做什么的？
  //* v1:参考点到每个点的向量
  //* v2:v1与主方向向量的叉积
  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    if ((j >= pl.size()) || (i_cur >= pl.size()))
      break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    //* 叉积v2的大小反映了点到主方向向量的垂直距离
    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    //* leng_wid记录了最大垂直距离的平方
    if (lw > leng_wid)
    {
      leng_wid = lw;
    }
  }
  //* 长度的平方与宽度的比值小于阈值，说明点分布不够“细长”    p2l_ratio = 225
  if ((two_dis * two_dis / leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;      //* return 0不是平面
  }

  //* 对disarr数组进行降序排列  disarr存储的是两点之间的距离
  //* 最慢的排序法了 可以优化！
  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  //* 距离太小说明存在异常 点分布不均？
  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }
  //* 
  if (lidar_type == AVIA)
  {
    //* 计算最大值与中间值的比例
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    //* 计算中间值与最小值的比例
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];
    //* 如果任一比率超过阈值，则不是平面
    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    //* 计算最大值与倒数第二小值的比例
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  //* curr_direct是当前判定为平面的主方向向量
  curr_direct << vx, vy, vz;
  curr_direct.normalize();    //* 方向向量归一化
  return 1;    //* 返回平面值
}

//* 过滤掉不可靠的边缘点，保留真正代表物体的边缘点  0可以认为当前点是右边缘点   1可以认为是左边缘点？
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  //* 前向判断
  //* 相邻点存在盲区，则直接返回false
  if (nor_dir == 0)
  {
    if (types[i - 1].range < blind || types[i - 2].range < blind)
    {
      return false;
    }
  }
  //* 后向判断
  else if (nor_dir == 1)
  {
    if (types[i + 1].range < blind || types[i + 2].range < blind)
    {
      return false;
    }
  }
  //* nor_dir = 0时，是i-1和i-2     nor_dir = 1时,是i 和 i+1
  //* i-1 <----> i-2 <---->cur               cur <---->i+1 <-----> i+2
  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;

  //* 始终保持d1是较大的那个
  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  //* 距离比例阈值 和距离差阈值  
  //* edgea = 2  edgeb = 0.1   总的来说就是这两个点不能距离太远
  if (d1 > edgea * d2 || (d1 - d2) > edgeb)
  {
    return false;
  }
  //* 都符合条件才能返回true
  return true;
}

void Preprocess::rs_handler(const sensor_msgs::PointCloud2_<allocator<void>>::ConstPtr &msg)
{
  pl_surf.clear();

  pcl::PointCloud<rslidar_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  pl_surf.reserve(plsize);

  /*** These variables only works when no point timestamps given ***/
  double omega_l = 0.361 * SCAN_RATE; // scan angular velocity
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);   // yaw of first scan point
  std::vector<float> yaw_last(N_SCANS, 0.0);  // yaw of last scan point
  std::vector<float> time_last(N_SCANS, 0.0); // last offset time
  /*****************************************************************/

  if (pl_orig.points[plsize - 1].timestamp > 0) // todo check pl_orig.points[plsize - 1].time
  {
    given_offset_time = true;
    // std::cout << "given_offset_time = true " << std::endl;
  }
  else
  {
    given_offset_time = false;
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578; // 记录第一个点(index 0)的yaw， to degree
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring; // 第一个点(index 0)的layer序号
    for (uint i = plsize - 1; i > 0; i--)     // 倒序遍历，找到与第一个点相同layer的最后一个点
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578; // 与第一个点相同layer的最后一个点的yaw
        break;
      }
    }
  }

  for (int i = 0; i < plsize; i++)
  {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) * 1000.0; // curvature unit: ms
    // std::cout << "added_pt.curvature:" << added_pt.curvature << std::endl;

    if (!given_offset_time)
    {
      int layer = pl_orig.points[i].ring;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[layer])
      {
        // printf("layer: %d; is first: %d", layer, is_first[layer]);
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.curvature = 0.0;
        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
        continue;
      }

      // compute offset time
      if (yaw_angle <= yaw_fp[layer])
      {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
      }
      else
      {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
      }

      if (added_pt.curvature < time_last[layer])
        added_pt.curvature += 360.0 / omega_l;

      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.curvature;
    }

    if (i % point_filter_num == 0)
    {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind) )
      {
        pl_surf.points.push_back(added_pt);
      }
    }
  }
}
