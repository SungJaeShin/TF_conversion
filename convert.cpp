#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <istream>
#include <typeinfo>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

std::vector<double> parseLine(std::istream &file)
{
    std::vector<double> parse;
    std::string line;
    
    // Get Line
    std::getline(file, line);
    std::stringstream s_line(line);
    
    // Get Values
    double val;
    while(s_line >> val)
    {
        parse.push_back(val);
    }

    return parse;
}

Eigen::Matrix4f _4x4Matrix(std::vector<double> values)
{
    // Get Position and Orientation
    Eigen::Quaterniond q;
    q.x() = values[4];
    q.y() = values[5];
    q.z() = values[6];
    q.w() = values[7];
    Eigen::Matrix3d m = q.normalized().toRotationMatrix();

    // Convert 4x4 GT TF Matrix
    Eigen::Matrix4f _4x4Matrix = Eigen::Matrix4f::Identity();
    // Put Orientation
    _4x4Matrix(0, 0) = m(0, 0);
    _4x4Matrix(0, 1) = m(0, 1);
    _4x4Matrix(0, 2) = m(0, 2);
    _4x4Matrix(1, 0) = m(1, 0);
    _4x4Matrix(1, 1) = m(1, 1);
    _4x4Matrix(1, 2) = m(1, 2);
    _4x4Matrix(2, 0) = m(2, 0);
    _4x4Matrix(2, 1) = m(2, 1);
    _4x4Matrix(2, 2) = m(2, 2);
    _4x4Matrix(3, 3) = 1;    
    // Put Position 
    _4x4Matrix(0, 3) = values[1];
    _4x4Matrix(1, 3) = values[2];
    _4x4Matrix(2, 3) = values[3];

    return _4x4Matrix;
}

std::vector<double> _vector8(double time, Eigen::Matrix4f matrix)
{
    // Convert Matrix to Quaternion
    std::vector<double> tmp;
    Eigen::Matrix3f mat;
    mat(0, 0) = matrix(0, 0);
    mat(0, 1) = matrix(0, 1);
    mat(0, 2) = matrix(0, 2);
    mat(1, 0) = matrix(1, 0);
    mat(1, 1) = matrix(1, 1);
    mat(1, 2) = matrix(1, 2);
    mat(2, 0) = matrix(2, 0);
    mat(2, 1) = matrix(2, 1);
    mat(2, 2) = matrix(2, 2);
    Eigen::Quaternionf q(mat);

    // Put Tum file format
    tmp.push_back(time);
    tmp.push_back(matrix(0, 3));
    tmp.push_back(matrix(1, 3));
    tmp.push_back(matrix(2, 3));
    tmp.push_back(q.x());
    tmp.push_back(q.y());
    tmp.push_back(q.z());
    tmp.push_back(q.w());

    return tmp;
}

int main()
{
    // Get Files
    std::ifstream gt, local;
    gt.open("~/MH01_rpg_gt.txt");
    local.open("~/vio_loop_map_MH_01.tum");

    // Get first GT Pose
    // TUM files format: time x y z qx qy qz qw
    std::vector<double> gt_first = parseLine(gt);
    Eigen::Matrix4f GT_Matrix = _4x4Matrix(gt_first);
    std::cout << "[First GT Matrix] \n" << GT_Matrix << std::endl;

    // Get first Local Pose
    // TUM files format: time x y z qx qy qz qw
    std::vector<double> local_first = parseLine(local);
    Eigen::Matrix4f Local_Matrix = _4x4Matrix(local_first);
    std::cout << "[First Local Matrix] \n" << Local_Matrix << std::endl;

    // Get TF world_T_local Matrix
    Eigen::Matrix4f First_TF = GT_Matrix * Local_Matrix.transpose();
    std::cout << "[First TF Matrix] \n" << First_TF << std::endl;
    std::vector<double> first_pose = _vector8(local_first[0], First_TF);

    // Write TF Pose
    std::ofstream tf_pose;
    tf_pose.open("/home/sj/workspace/etri_ws/result/result/TF_VIO.tum");
    for(int i = 0; i < first_pose.size(); i++)
    {
        std::string tmp_value = std::to_string(first_pose[i]);
        tf_pose.write(tmp_value.c_str(), tmp_value.size());
        tf_pose.write(" ", 1);
    }

    // Get TF Pose
    while(!local.eof())
    {
        std::vector<double> tmp_values = parseLine(local);
        Eigen::Matrix4f tmp_matrix = _4x4Matrix(tmp_values);
        Eigen::Matrix4f world_pose = First_TF * tmp_matrix;
        std::vector<double> write_pose = _vector8(tmp_values[0], world_pose);

        for(int i = 0; i < write_pose.size(); i++)
        {
            std::string tmp_value = std::to_string(write_pose[i]);
            tf_pose.write(tmp_value.c_str(), tmp_value.size());
            if(i + 1 != write_pose.size())
                tf_pose.write(" ", 1);
        }
        tf_pose.write("\n", 1);
    }

    gt.close();
    local.close();
    tf_pose.close();

    return 0;
}
