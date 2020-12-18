#ifndef  MEIGEN_H
#define  MEIGEN_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>  // Eigen 几何模块

namespace meigen {

/**
 * @brief   获取空间向量夹角
 * @param  [in]  vecotor1   向量1
 * @param  [in]  vecotor2   向量2
 * @retval   angle 夹角角度
 * @note 返回值0-180,不能表示具体方向
 */
static double getAngleBetweenVecotor3D( const Eigen::Vector3d& v1,
                                        const Eigen::Vector3d& v2 ) {
    //计算向量间內积
    double dot = v1.dot( v2 );
    //计算模
    double mould1 = v1.norm();
    double mould2 = v2.norm();
    //MLOGD("mould1: %lf mould2: %lf.", mould1, mould2 );
    //计算夹角弧度
    double radian = acos( dot / ( mould1 * mould2 ) );
    //弧度转角度
    return radian / M_PI * 180;
}

//获取平面向量夹角
//单位弧度
static double GetAngleBetweenVecotor2D( const Eigen::Vector2d& v1,
                                        const Eigen::Vector2d& v2 ) {
    //计算向量间內积
    double dot = v1.dot( v2 );
    //计算模
    double mould1 = v1.norm();
    double mould2 = v2.norm();
    //MLOGD("mould1: %lf mould2: %lf.", mould1, mould2 );
    //计算夹角弧度
    double radian = acos( dot / ( mould1 * mould2 ) );
    return  radian;
}

//已知坐标变换，求变换坐标系下一点在原坐标系中的映射
//先平移，坐标系重合
//再加上当前坐标系下点映射
///参数:
///1.原坐标系到当前坐标系旋转 Q
///2.原坐标系到当前坐标系平移 T
///3.目标点在当前坐标系下坐标
static Eigen::Vector3d pointToOriginCoordinate( const Eigen::Quaterniond& Q,
                                                const Eigen::Vector3d& T,
                                                const Eigen::Vector3d& inP ){
        Eigen::Vector3d outP;
        //平移操作
        outP.x() = T.x();
        outP.y() = T.y();
        outP.z() = T.z();
        //旋转操作求映射
        Eigen::Quaterniond qTPCAM= Q;
        Eigen::Quaterniond pqTurn;  //( w,x,y,z)
        Eigen::Quaterniond pqOrigin( 0, inP.x(), inP.y(), inP.z() );  //( w,x,y,z)

        pqTurn  = qTPCAM.inverse() * pqOrigin * qTPCAM;

        outP.x() += pqTurn.x();
        outP.y() += pqTurn.y();
        outP.z() += pqTurn.z();
        return  outP;
}


//已知坐标变换，求变换坐标系下的点在新坐标系中的映射
//先平移，坐标系重合
//再加上当前坐标系下点映射
///参数:
///1.新坐标系旋转相对当前坐标系旋转 Q
///2.新坐标系平移相对当前坐标系平移 T
///3.目标点在当前坐标系下坐标
static Eigen::Vector3d PointToNewCoordinate( const Eigen::Quaterniond& Q,
                                             const Eigen::Vector3d& T,
                                             const Eigen::Vector3d& inP ){
        Eigen::Vector3d outP;
        //平移操作
        outP.x() -= T.x();
        outP.y() -= T.y();
        outP.z() -= T.z();

        //旋转操作求映射
        Eigen::Quaterniond qTPCAM= Q;
        Eigen::Quaterniond pqTurn;  //( w,x,y,z)
        Eigen::Quaterniond pqOrigin( 0, inP.x(), inP.y(), inP.z() );  //( w,x,y,z)

        pqTurn  = qTPCAM * pqOrigin * qTPCAM.inverse();

        outP.x() += pqTurn.x();
        outP.y() += pqTurn.y();
        outP.z() += pqTurn.z();
        return  outP;
}



///四元素转旋转向量
static Eigen::AngleAxisd QuaterniondToAngleAxisd( const Eigen::Quaterniond& q ) {
    Eigen::AngleAxisd rotation_vector( q );
    return rotation_vector;
}

//RotationRollPitchYaw To Quaterniond
//存储顺序为 ypr[ 0 ] = 绕Z ypr[1] = 绕Y ypr[2]  = 绕X
//note: Rotation ypr equal to Euler ZYX
//ypr --- y * p * r  : left take
static Eigen::Quaterniond RotationYPRToQuaterniond( const double yaw, const double pitch, const double roll ) {
    Eigen::Quaterniond q;
    Eigen::Matrix3d R;
    R   =   Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd( pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd( roll, Eigen::Vector3d::UnitX());
    q = R;
    return q;
}

static Eigen::Quaterniond RotationYPRToQuaterniond( const Eigen::Vector3d& ypr ) {
    Eigen::Quaterniond q;
    Eigen::Matrix3d R;
    R   =   Eigen::AngleAxisd( ypr[0], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd( ypr[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd( ypr[2], Eigen::Vector3d::UnitX());
    q = R;
    return q;
}


////ypr-ZYX
static Eigen::Matrix3d RotationYPRToRotationMatrix( const double yaw, const double pitch, const double roll ){

  Eigen::AngleAxisd yawAngle( Eigen::AngleAxisd( yaw ,Eigen::Vector3d::UnitZ() ) );
  Eigen::AngleAxisd pitchAngle( Eigen::AngleAxisd( pitch ,Eigen::Vector3d::UnitY() ) );
  Eigen::AngleAxisd rollAngle( Eigen::AngleAxisd( roll , Eigen::Vector3d::UnitX()));

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = yawAngle * pitchAngle * rollAngle;
  return rotation_matrix;

}

/**
 * @brief   Quaterniond To Rotation YPR
 * @param  [in]  Eigen Quaterniond
 * @retval      Rotation ypr Vector3d 按照(Y,P,R)( Z, Y, X ) 方式存储
 *              返回的值范围[ 0 -- +Pi ] [ -Pi -- +Pi ] [ -Pi -- +Pi ]（ 四元素归一化问题导致 ），不适合用来提取航向角
 * @note        Rotation ypr Vector3d 按照(Y,P,R)( Z, Y, X ) 方式存储,即ypr[0]=yaw ypr[1]=pitch ypr[2]=roll
 */
static Eigen::Vector3d QuaterniondToRotationYPR( const Eigen::Quaterniond& q ) {
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    Eigen::Vector3d ypr = rotation_matrix.eulerAngles(2,1,0);
    return ypr;
}

/**
 * @brief   Quaterniond To Rotation YPR
 * @param  [in]  Eigen Quaterniond
 * @retval      Rotation ypr Vector3d 按照(Y,P,R)( Z, Y, X ) 方式存储
 *              返回的值范围[ -Pi -- +Pi ] [ -Pi/2 -- +Pi/2 ] [ -Pi -- +Pi ]，适合用来提取航向角
 * @note        Rotation ypr Vector3d 按照(Y,P,R)( Z, Y, X ) 方式存储,即ypr[0]=yaw ypr[1]=pitch ypr[2]=roll
 */
static Eigen::Vector3d QuaterniondToRotationYPR_Y( const Eigen::Quaterniond& q ) {

    Eigen::Vector3d ans;
    //double q2sqr = data.q2 * data.q2;
    double q2sqr = q.y() * q.y();

    double t0 = -2.0 * (q2sqr + q.z() * q.z() ) + 1.0;

    double t1 = +2.0 * (q.x() * q.y() + q.w() * q.z() );
    double t2 = -2.0 * (q.x() * q.z() - q.w() * q.y() );
    double t3 = +2.0 * (q.y() * q.z() + q.w() * q.x() );
    double t4 = -2.0 * (q.x() * q.x() + q2sqr ) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    ans[ 0 ] = atan2(t1, t0);   ///yaw
    ans[ 1 ] = asin(t2);        ///pitch
    ans[ 2 ] = atan2(t3, t4);   ///roll

    return ans;
}

static Eigen::Quaterniond RotationYPRToQuaterniond_Y( const Eigen::Vector3d &ypr ) {

    Eigen::Quaterniond q;

    double yaw = ypr[ 0 ];
    double pitch = ypr[ 1 ];
    double roll = ypr[ 2 ];

    double t0 = cos( yaw * 0.5);
    double t1 = sin( yaw * 0.5);
    double t2 = cos( roll * 0.5);
    double t3 = sin( roll * 0.5);
    double t4 = cos( pitch * 0.5);
    double t5 = sin( pitch * 0.5);

    q.w() = t2 * t4 * t0 + t3 * t5 * t1;
    q.x() = t3 * t4 * t0 - t2 * t5 * t1;
    q.y() = t2 * t5 * t0 + t3 * t4 * t1;
    q.z() = t2 * t4 * t1 - t3 * t5 * t0;
    return q;

}

//求新坐标系下点坐标,而不是在原坐标系下旋转得到的坐标（应该叫仿射射变换 Affine）
//q为当前坐标系到原坐标旋转变换
static Eigen::Vector3d  Vector3dTransformByQuaterniond( const Eigen::Vector3d& in, const Eigen::Quaterniond& q ){
        Eigen::Quaterniond qIn( 0, in.x() , in.y(), in.z() );  //( w,x,y,z)
        Eigen::Quaterniond qTurn;
        //qTurn  = q * qIn * q.inverse();
        qTurn  = q.inverse() * qIn * q;
        Eigen::Vector3d out;
        out.x() = qTurn.x();
        out.y() = qTurn.y();
        out.z() = qTurn.z();
        return out;
}


//将四元数绕v轴旋转p度,相对世界坐标系
static Eigen::Quaterniond QuaterniondAxisRotateWorld( const Eigen::Quaterniond& q, const Eigen::Vector3d& v , const double& p ) {
    Eigen::AngleAxisd rotation_vector ( p, v);
    Eigen::Quaterniond q_r = Eigen::Quaterniond ( rotation_vector );
    Eigen::Quaterniond n_q;
    n_q = q_r * q; //左乘绕世界坐标旋转，右乘绕自身,这里是左乘
    return n_q;
}

//求一组向量的均值
static Eigen::Vector3d GetVetors3dMean( const std::vector<Eigen::Vector3d>& vectors ){
    if( vectors.empty() ){
        return Eigen::Vector3d(0,0,0);
    }
    Eigen::Vector3d sum_v(0,0,0);
    Eigen::Vector3d mean_v;
    for( auto it = vectors.begin(); it < vectors.end() ; it++ ) {
        sum_v += *it;
    }
    mean_v = sum_v / vectors.size();
    return  mean_v;
}


//求一组向量的方差
static Eigen::Vector3d GetVetors3dVariance( const std::vector<Eigen::Vector3d>& vectors ){
    Eigen::Vector3d s;
    Eigen::Vector3d mean_v = GetVetors3dMean( vectors );
    Eigen::Vector3d sum_var(0, 0, 0);
    for( auto it = vectors.begin(); it < vectors.end() ; it++ ) {
        sum_var.x() += std::pow( ( it->x() - mean_v.x() ), 2 );
        sum_var.y() += std::pow( ( it->y() - mean_v.y() ), 2 );
        sum_var.z() += std::pow( ( it->z() - mean_v.z() ), 2 );
    }
    s = sum_var / ( vectors.size() - 1 );  //无偏估计
    return  s;
}


//求一组向量的标准差
static Eigen::Vector3d GetVetors3dStandardDeviation( const std::vector<Eigen::Vector3d>& vectors ){
    Eigen::Vector3d s;
    Eigen::Vector3d sd;
    s = GetVetors3dVariance( vectors );
    sd.x() = std::sqrt( s.x() );
    sd.y() = std::sqrt( s.y() );
    sd.z() = std::sqrt( s.z() );
    return  sd;
}


} //namespace meigen  end

#endif


//test
#if 0

void meigen_test( void ){
  Eigen::Vector3d v1(1,0,0);    //原始坐标系下点
  Eigen::Vector3d ypr( M_PI / 4 , 0 , 0 );
  Eigen::Vector3d ypr1( M_PI / 4 , 0 , 0 );
  Eigen::Quaterniond q,q1;
  q = meigen::RotationYPRToQuaterniond( ypr );
  q1 = meigen::RotationYPRToQuaterniond( ypr1 );
  Eigen::Quaterniond mq = q1 * q;
  Eigen::Matrix3d  R = mq.matrix();    //旋转矩阵
  //Eigen::AngleAxisd rotation_vector( M_PI/6, Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 45 度
  //Eigen::Matrix3d  R( rotation_vector );    //旋转矩阵

  Eigen::Vector3d v2 = meigen::Vector3dTransformByQuaterniond( v1, mq );
  //Eigen::Vector3d v2 = R.inverse() * v1;
  cout << v2 << endl;   //新坐标系下点坐标,而不是在原坐标系下旋转（应该叫放射变换）

  Eigen::Vector3d v2_back = meigen::Vector3dTransformByQuaterniond( v2, mq.inverse() );

  cout << v2_back << endl;

}
#endif


