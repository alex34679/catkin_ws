#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>


Eigen::Quaterniond modifyYaw(Eigen::Quaterniond quat, double new_yaw) {
    // 将四元数转换为欧拉角
    Eigen::Vector3d euler_angles = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    
    // 设置新的yaw值
    euler_angles[2] = new_yaw;
    
    // 创建新的旋转矩阵
    Eigen::Matrix3d new_rotation_matrix;
    new_rotation_matrix = Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX());
    
    // 创建新的四元数
    Eigen::Quaterniond new_quaternion(new_rotation_matrix);
    
    return new_quaternion;
}

// 你提供的函数定义
Eigen::Quaterniond calculateQuaternion(const Eigen::Vector3d& cur_path, double fai) {  
    const double g = 9.8;    
    Eigen::Vector3d t = cur_path.head<3>(); // 使用Eigen的head方法获取前三个元素  
    t[2] += g; // 更新z分量  

    // 归一化t向量得到Z_b  
    Eigen::Vector3d Z_b = t.normalized();  
    // Eigen::Vector3d Z_b = t / std::sqrt(t.x() * t.x() + t.y() * t.y() + t.z() * t.z());
    // 计算X_c  
    Eigen::Vector3d X_c(cos(fai), sin(fai), 0);  

    // 计算Y_b  
    Eigen::Vector3d Y_b = Z_b.cross(X_c).normalized();  
    // 计算X_b  
    Eigen::Vector3d X_b = Y_b.cross(Z_b);  
    
    // 构造旋转矩阵  
    Eigen::Matrix3d rota_matrix;  
    rota_matrix << X_b, Y_b, Z_b;   
    std::cout << "X_b:\n" << X_b << std::endl;
    std::cout << "Y_b:\n" << Y_b << std::endl;
    std::cout << "Z_b:\n" << Z_b << std::endl;
    Eigen::Quaterniond quaternion(rota_matrix);



    return quaternion;  
}


// Eigen::Quaterniond calculateQuaternion(const Eigen::Vector3d& thrust, double fai) {
//     // 归一化 thrust 向量，计算 z_b
//     Eigen::Vector3d z_b = thrust.normalized();

//     // 定义 e_z 向量
//     Eigen::Vector3d e_z(0.0, 0.0, 1.0);

//     // 计算四元数的实部 q_w
//     double q_w = 1.0 + e_z.dot(z_b);

//     // 计算四元数的虚部 q_xyz
//     Eigen::Vector3d q_xyz = e_z.cross(z_b);

//     // 组合四元数 q
//     Eigen::Quaterniond q(q_w, q_xyz.x(), q_xyz.y(), q_xyz.z());

//     // 归一化四元数
//     q.normalize();

//     return q;
// }

int main() {
    // 定义路径列表，生成从 0 到 10 范围内的路径向量
    std::vector<Eigen::Vector3d> path_list;

    for (int x = 0; x <= 10; ++x) {
        for (int y = 0; y <= 10; ++y) {
            for (int z = 0; z <= 10; ++z) {
                float xx = (float)x;
                float yy = (float)y;
                float zz = (float)z;
                path_list.emplace_back(xx, yy, zz);
            }
        }
    }

    double fai = 0.0;

    for (const auto& cur_path : path_list) {
        Eigen::Quaterniond quaternion = calculateQuaternion(cur_path, fai);
        // Eigen::Quaterniond modified_quat = modifyYaw(quaternion, 0.0);

        // 将四元数转为欧拉角
        Eigen::Vector3d euler_angles = quaternion.toRotationMatrix().eulerAngles(2, 0, 1); // 顺序为 ZYX
        
        double yaw = euler_angles[0]; // yaw 是第一个角（Z轴旋转）
        
        std::cout << "Cur path: [" << cur_path.transpose() << "], Yaw angle: " << yaw << std::endl;
    }

    return 0;
}
