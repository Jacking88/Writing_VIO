#include <iostream>
#include<Eigen/Core>
#include<Eigen/Geometry>
using namespace std;

int main(int argc, char **argv) {
    //变量初始化
    Eigen::Vector3d w(0.01,0.02,0.03);//小量角速度（旋转向量）
    Eigen::Matrix3d R=Eigen::AngleAxisd(M_PI/4,Eigen::Vector3d(0,0,1)).toRotationMatrix();//初始旋转矩阵，绕Z轴旋转45°
    Eigen::Quaterniond q(R);//初始四元数
    cout<<"初始旋转矩阵："<<endl;
    cout<<R<<endl;
    cout<<"初始四元数："<<endl;
    cout<<q.coeffs().transpose() <<endl;
    
    //利用Rodrigues's formula完成旋转矩阵更新
    double theta=w.norm();//旋转向量对应的旋转角
    Eigen::Vector3d n_w=w/theta;//归一化得到旋转向量对应的旋转轴
    Eigen::Matrix3d n_w_skew;
    n_w_skew<<   0,    -n_w(2),    n_w(1),
		n_w(2),     0,     -n_w(0),
	       -n_w(1),  n_w(0),      0;
    Eigen::Matrix3d R_w=cos(theta)*Eigen::Matrix3d::Identity()+(1-cos(theta))*n_w*n_w.transpose()+sin(theta)*n_w_skew;//Rodrigues's formula
    Eigen::Matrix3d R_update=R*R_w;
    cout<<"更新后的旋转矩阵："<<endl;
    cout<<R_update<<endl;    
    
    //四元数更新    
    Eigen::Quaterniond q_w(1,w(0)/2,w(1)/2,w(2)/2);//小量角速度对应的四元数
    Eigen::Quaterniond q_update=q*q_w;
    q_update=q_update.normalized();//单位四元数才可以表示三维旋转，所以必须归一化
    cout<<"更新后的四元数："<<endl;
    cout<<q_update.coeffs().transpose() <<endl;   
    
    //计算两种方法得到的结果之差
    cout<<"两种方法得到的结果之差："<<endl;    
    cout<<q_update.toRotationMatrix()-R_update<<endl;

    return 0;
}