//
// Created by nimapng on 7/20/21.
//


#include "MatFileHandler.h"

/* template <typename T>
bool SaveMatlabMat(T *src, string savePath, string matrixName, int cols, int rows)
{
    // tanspose befoee being saved
    int datasize = cols * rows;
    double *Final = new double[datasize]; //convert to double precision
    memset(Final, 0, datasize * sizeof(double));
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            Final[j * rows + i] = double(src[i * cols + j]);
        }
    }
    mxArray *pWriteArray = NULL; //matlab mat form
    MATFile *pmatFile = NULL;    //.mat file
    pmatFile = matOpen(savePath.c_str(), "u");
    if (pmatFile == nullptr)
        pmatFile = matOpen(savePath.c_str(), "w");
    if (pmatFile == nullptr)
    {
        printf("mat save path is error");
        return false;
    }
    pWriteArray = mxCreateDoubleMatrix(rows, cols, mxREAL);
    memcpy((void *)(mxGetPr(pWriteArray)), (void *)Final, sizeof(double) * datasize);
    matPutVariable(pmatFile, matrixName.c_str(), pWriteArray);

    matClose(pmatFile);          //close file
    mxDestroyArray(pWriteArray); //release resource
    delete[] Final;              //release resource
    Final = nullptr;
    return true;
} */

bool savemat(MatrixXd var, std::string file_name, std::string var_name) {
    size_t dims[2];
    dims[0] = var.rows();
    dims[1] = var.cols();
    mat_t *mat;
    mat = Mat_CreateVer(file_name.c_str(), NULL, MAT_FT_DEFAULT);
    if (!mat) {
        return false;
    }
    matvar_t *matvar;
    matvar = Mat_VarCreate(var_name.c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, var.data(), 0);
    Mat_VarWrite(mat, matvar, MAT_COMPRESSION_NONE);
    Mat_VarFree(matvar);
    Mat_Close(mat);
    return true;
}

bool saveVec(VectorXf var, std::string file_name, std::string var_name) {
    size_t dims[2];
    dims[0] = var.rows();
    dims[1] = 1;
    mat_t *mat;
    mat = Mat_Open(file_name.c_str(), MAT_ACC_RDWR);
    // if (!mat)
    // {
    //     mat = Mat_CreateVer(file_name.c_str(), NULL, MAT_FT_DEFAULT);
    //     if (!mat)
    //         return false;
    // }
    mat = Mat_CreateVer(file_name.c_str(), NULL, MAT_FT_DEFAULT);
    if (!mat)
        return false;
    matvar_t *matvar;
    matvar = Mat_VarCreate(var_name.c_str(), MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, var.data(), 0);
    Mat_VarWrite(mat, matvar, MAT_COMPRESSION_NONE);
    Mat_VarFree(matvar);
    Mat_Close(mat);
    return true;
}

bool readMat(const char *file, const char *name, Eigen::Ref<Eigen::Matrix<int, -1, -1>> mat) {
    mat_t *matfp;
    matvar_t *matvar;

    matfp = Mat_Open(file, MAT_ACC_RDONLY);
    if (NULL == matfp) {
        fprintf(stderr, "Error opening MAT file %s\n", file);
        return false;
    }

    matvar = Mat_VarRead(matfp, name);
//    std::cout << "mat data type: " << matvar->class_type << std::endl;

    if (matvar->rank != 2 || matvar->class_type != MAT_C_INT32) {
        return false;
    }
    // printf("mat size: %ld, %ld\n", matvar->dims[0], matvar->dims[1]);
    Eigen::Matrix<int32_t,-1,-1>_mat(matvar->dims[0], matvar->dims[1]);
    mat.resize(matvar->dims[0], matvar->dims[1]);
    std::memcpy(_mat.data(), (int32_t *) matvar->data, sizeof(int32_t) * _mat.size());
    mat = _mat.template cast<int>();
    /* for (int c = 0; c < matvar->dims[1]; c++)
    {
        for (int r = 0; r < matvar->dims[0]; r++)
        {
            mat(r, c) = ((double *)matvar->data)[r * matvar->dims[1] + c];
        }
    } */

    Mat_Close(matfp);
    return true;
}

bool readMat(const char *file, const char *name, Eigen::Ref<Eigen::Matrix<float, -1, -1>> mat) {
    mat_t *matfp;
    matvar_t *matvar;

    matfp = Mat_Open(file, MAT_ACC_RDONLY);
    if (NULL == matfp) {
        fprintf(stderr, "Error opening MAT file %s\n", file);
        return false;
    }

    matvar = Mat_VarRead(matfp, name);
    if (matvar->rank != 2 || matvar->class_type != MAT_C_DOUBLE) {
        return false;
    }
    // printf("mat size: %ld, %ld\n", matvar->dims[0], matvar->dims[1]);
    Eigen::MatrixXd _mat(matvar->dims[0], matvar->dims[1]);
    mat.resize(matvar->dims[0], matvar->dims[1]);
    std::memcpy(_mat.data(), (double *) matvar->data, sizeof(double) * _mat.size());
    mat = _mat.template cast<float>();
    /* for (int c = 0; c < matvar->dims[1]; c++)
    {
        for (int r = 0; r < matvar->dims[0]; r++)
        {
            mat(r, c) = ((double *)matvar->data)[r * matvar->dims[1] + c];
        }
    } */

    Mat_Close(matfp);
    return true;
}

bool readMat(const char *file, const char *name, Eigen::Ref<Eigen::Matrix<double, -1, -1>> mat) {
    mat_t *matfp;
    matvar_t *matvar;

    matfp = Mat_Open(file, MAT_ACC_RDONLY);
    if (NULL == matfp) {
        fprintf(stderr, "Error opening MAT file %s\n", file);
        return false;
    }

    matvar = Mat_VarRead(matfp, name);
    if (matvar->rank != 2 || matvar->class_type != MAT_C_DOUBLE) {
        return false;
    }
    // printf("mat size: %ld, %ld\n", matvar->dims[0], matvar->dims[1]);
    mat.resize(matvar->dims[0], matvar->dims[1]);
    std::memcpy(mat.data(), (double *) matvar->data, sizeof(double) * mat.size());
    /* for (int c = 0; c < matvar->dims[1]; c++)
    {
        for (int r = 0; r < matvar->dims[0]; r++)
        {
            mat(r, c) = ((double *)matvar->data)[r * matvar->dims[1] + c];
        }
    } */

    Mat_Close(matfp);
    return true;
}


