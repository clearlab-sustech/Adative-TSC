//
// Created by wenchun on 4/13/21.
//

#ifndef MAT_FILE_HANDLER_H
#define MAT_FILE_HANDLER_H

#include <string.h>
#include <iostream>
#include <stdio.h>
#include <eigen3/Eigen/Core>
#include <matio.h>

using namespace Eigen;

using namespace std;

template<typename T>
bool SaveMatlabMat(T *src, string savePath, string matrixName, int cols, int rows);

bool savemat(MatrixXd var, std::string file_name, std::string var_name);

bool saveVec(VectorXf var, std::string file_name, std::string var_name);

bool readMat(const char *file, const char *name, Eigen::Ref<Eigen::Matrix<int, -1, -1>> mat);

bool readMat(const char *file, const char *name, Eigen::Ref<Eigen::Matrix<float, -1, -1>> mat);

bool readMat(const char *file, const char *name, Eigen::Ref<Eigen::Matrix<double, -1, -1>> mat);

#endif // MAT_FILE_HANDLER_H

